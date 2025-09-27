/*
 * Copyright (c) 2024 Light Bar System
 * SPDX-License-Identifier: Apache-2.0
 * 
 * BLE Light Bar Peripheral with WS2812 LED Strip
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/settings/settings.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>

#define STRIP_NUM_PIXELS 44
#define ACTIVE_PIXELS    44

#define STACKSIZE 1024
#define PRIORITY 7

/* Custom UUIDs for Light Bar Service */
#define BT_UUID_LIGHTBAR_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x47484649, 0x4341, 0x4244, 0x4140, 0x4345abcdef12)
#define BT_UUID_LIGHTBAR_CHAR_VAL \
	BT_UUID_128_ENCODE(0x47484649, 0x4341, 0x4244, 0x4140, 0x4345abcdef13)

static struct bt_uuid_128 lightbar_service_uuid = BT_UUID_INIT_128(BT_UUID_LIGHTBAR_SERVICE_VAL);
static struct bt_uuid_128 lightbar_char_uuid    = BT_UUID_INIT_128(BT_UUID_LIGHTBAR_CHAR_VAL);

/* LED strip device */
static const struct device *strip = DEVICE_DT_GET(DT_NODELABEL(led_strip));

/* Status LED */
#define LED_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* Work for LED blinking */
static struct k_work_delayable blink_work;
static bool blinking = false;

/* Data structure for light control */
struct lightbar_cmd {
	int32_t color;      // 0-8 (0=stop, 1-8=colors)
	int32_t brightness; // 1-100%
	int32_t amplitude;  // 1-100%
	int32_t frequency;  // 30-100 BPM
};

/* Current command */
static struct lightbar_cmd current_cmd = {0, 50, 50, 60};

/* Message queue for commands */
K_MSGQ_DEFINE(lightbar_queue, sizeof(struct lightbar_cmd), 16, 4);

/* LED strip pixels */
static struct led_rgb pixels[STRIP_NUM_PIXELS];

/* Color definitions */
static const struct led_rgb colors[] = {
	{0, 0, 0},       // 0: OFF
	{255, 0, 255},   // 1: Magenta
	{0, 0, 255},     // 2: Blue
	{0, 255, 0},     // 3: Green
	{0, 255, 255},   // 4: Cyan (duplicate)
	{255, 20, 147},  // 5: Pink
	{255, 128, 0},   // 6: Orange
	{255, 0, 0},     // 7: Red
	{255, 180, 80}  // 8: White
};

/* Advertising data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		(CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
		(CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LIGHTBAR_SERVICE_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Work for advertising */
static struct k_work adv_work;

/* ---------- LED Status Blink Handlers ---------- */
static void blink_handler(struct k_work *work)
{
	static bool on = false;

	if (!blinking) {
		gpio_pin_set_dt(&led, 1); // solid ON when connected
		return;
	}

	gpio_pin_set_dt(&led, on);
	on = !on;

	k_work_reschedule(&blink_work, K_MSEC(500)); // blink every 500ms
}

static void blink_start(void)
{
	blinking = true;
	k_work_reschedule(&blink_work, K_NO_WAIT);
}

static void blink_stop(void)
{
	blinking = false;
	gpio_pin_set_dt(&led, 1); // solid ON when connected
}

/* ---------- GATT Service ---------- */
static ssize_t lightbar_char_write(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len,
				uint16_t offset, uint8_t flags)
{
	if (offset + len > sizeof(struct lightbar_cmd)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	
	if (len != sizeof(struct lightbar_cmd)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	
	memcpy(&current_cmd, buf, len);
	
	/* Validate and constrain values */
	if (current_cmd.color < 0) current_cmd.color = 0;
	if (current_cmd.color > 8) current_cmd.color = 8;
	if (current_cmd.brightness < 1) current_cmd.brightness = 1;
	if (current_cmd.brightness > 100) current_cmd.brightness = 100;
	if (current_cmd.amplitude < 1) current_cmd.amplitude = 1;
	if (current_cmd.amplitude > 100) current_cmd.amplitude = 100;
	if (current_cmd.frequency < 30) current_cmd.frequency = 30;
	if (current_cmd.frequency > 100) current_cmd.frequency = 100;
	
	printk("Received: color=%d, brightness=%d, amplitude=%d, frequency=%d\n",
		current_cmd.color, current_cmd.brightness, current_cmd.amplitude, current_cmd.frequency);
	
	k_msgq_put(&lightbar_queue, &current_cmd, K_NO_WAIT);
	
	return len;
}

static ssize_t lightbar_char_read(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &current_cmd, sizeof(current_cmd));
}

BT_GATT_SERVICE_DEFINE(lightbar_service,
	BT_GATT_PRIMARY_SERVICE(&lightbar_service_uuid),
	BT_GATT_CHARACTERISTIC(&lightbar_char_uuid.uuid,
			      BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			      BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			      lightbar_char_read, lightbar_char_write, NULL),
);

/* ---------- Advertising ---------- */
static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

/* ---------- LED Helpers ---------- */
static void clear_strip(void)
{
	memset(pixels, 0, sizeof(pixels));
	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
}

/* Fun boot-up animation */
// static void startup_animation(void)
// {
// 	printk("Running startup animation...\n");
//   int j = 1;
// 	for (int i = 0; i < ACTIVE_PIXELS; i++) {
// 		// for (int j = 1; j < ARRAY_SIZE(colors); j++) {
//     if (j > ARRAY_SIZE(colors)){
//       j = 1;
//     } 
//     memset(pixels, 0, sizeof(pixels));
//     pixels[i] = colors[j];
//     led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
//     j++;
//     k_sleep(K_MSEC(30));
// 	}
// 	clear_strip();
// }


static void startup_animation(void)
{
	printk("Running startup animation...\n");
	
	int center = STRIP_NUM_PIXELS / 2;
	int max_spread = (STRIP_NUM_PIXELS / 2) + 1;
	
	/* Rich golden color */
	struct led_rgb golden = {255, 180, 80};
	
	/* Phase 1: Expand from center while brightening */
	for (int spread = 0; spread <= max_spread; spread++) {
		memset(pixels, 0, sizeof(pixels));
		
		/* Brightness increases as we expand */
		float brightness = (float)spread / max_spread;
		
		for (int i = 0; i <= spread; i++) {
			int left_pos = center - i;
			int right_pos = center + i;
			
			/* All lit pixels have same brightness */
			if (left_pos >= 0) {
				pixels[left_pos].r = (int)(golden.r * brightness);
				pixels[left_pos].g = (int)(golden.g * brightness);
				pixels[left_pos].b = (int)(golden.b * brightness);
			}
			if (right_pos < STRIP_NUM_PIXELS && right_pos != left_pos) {
				pixels[right_pos].r = (int)(golden.r * brightness);
				pixels[right_pos].g = (int)(golden.g * brightness);
				pixels[right_pos].b = (int)(golden.b * brightness);
			}
		}
		
		led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		k_sleep(K_MSEC(30));
	}
	
	/* Phase 2: Dim and brighten once (breathing effect) */
	/* Dim down */
	for (int b = 100; b >= 20; b -= 3) {
		for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
			pixels[i].r = (golden.r * b) / 100;
			pixels[i].g = (golden.g * b) / 100;
			pixels[i].b = (golden.b * b) / 100;
		}
		led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		k_sleep(K_MSEC(20));
	}
	
	/* Brighten back up */
	for (int b = 20; b <= 100; b += 2) {
		for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
			pixels[i].r = (golden.r * b) / 100;
			pixels[i].g = (golden.g * b) / 100;
			pixels[i].b = (golden.b * b) / 100;
		}
		led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		k_sleep(K_MSEC(20));
	}
	
	/* Hold briefly at full brightness */
	k_sleep(K_MSEC(200));
	
	/* Phase 3: Fade out */
  for (int b = 100; b >= 0; b -= 2) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
      pixels[i].r = (golden.r * b) / 100;
      pixels[i].g = (golden.g * b) / 100;
      pixels[i].b = (golden.b * b) / 100;
    }
    led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
    k_sleep(K_MSEC(20));
  }
	
	clear_strip();
	printk("Startup animation complete\n");
}

/* ---------- Connection callbacks ---------- */
static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Failed to connect to %s, 0x%02x %s\n", addr, conn_err,
		       bt_hci_err_to_str(conn_err));
		return;
	}

	printk("Connected: %s\n", addr);
	printk("Connection established after advertising restart\n");

	blink_stop(); // Solid LED when connected
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

	/* Clear LED strip immediately */
	clear_strip();

	/* Reset to default state */
	struct lightbar_cmd reset_cmd = {0, 50, 50, 60};
	current_cmd = reset_cmd;
	k_msgq_put(&lightbar_queue, &reset_cmd, K_NO_WAIT);

	/* Restart advertising */
	advertising_start();
	blink_start();

	printk("Advertising restarted after disconnect\n");
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d %s\n",
		       addr, level, err, bt_security_err_to_str(err));
	}
}

static void recycled_cb(void)
{
	printk("Connection object recycled, advertising restarted\n");
	advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
	.recycled = recycled_cb,
};

/* ---------- Auth callbacks ---------- */
static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing failed conn: %s, reason %d %s\n", addr, reason,
	       bt_security_err_to_str(reason));
}

static const struct bt_conn_auth_cb auth_callbacks = {.cancel = auth_cancel};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

/* ---------- LED Strip Control Thread ---------- */
static void update_strip(int position, int amplitude, struct led_rgb color, int brightness)
{
	memset(pixels, 0, sizeof(pixels));

	int num_pixels = (ACTIVE_PIXELS * amplitude) / 100;
	if (num_pixels < 1) num_pixels = 1;
	if (num_pixels > ACTIVE_PIXELS) num_pixels = ACTIVE_PIXELS;

	struct led_rgb adjusted_color;
	adjusted_color.r = (color.r * brightness) / 100;
	adjusted_color.g = (color.g * brightness) / 100;
	adjusted_color.b = (color.b * brightness) / 100;

	int start_pos = position;
	for (int i = 0; i < num_pixels && (start_pos + i) < ACTIVE_PIXELS; i++) {
		pixels[start_pos + i] = adjusted_color;
	}

	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
}

// static void lightbar_thread(void)
// {
// 	struct lightbar_cmd cmd;
// 	int position = 0;
// 	int direction = 1;
// 	// int64_t last_update = 0;


	// while (1) {
		// if (k_msgq_get(&lightbar_queue, &cmd, K_NO_WAIT) == 0) {
		// 	if (cmd.color == 0) {
		// 		clear_strip();
		// 		k_msgq_get(&lightbar_queue, &cmd, K_FOREVER);
		// 	}
		// } else {
		// 	cmd = current_cmd;
		// }
  //   float beat_ms = 60000 / cmd.frequency;
  //   float step_ms = beat_ms / ACTIVE_PIXELS;
		
	// 	if (cmd.color > 0 && cmd.color <= 8) {
	// 		// int beat_ms = 60000 / cmd.frequency;
	// 		// // int step_ms = beat_ms / (2 * (ACTIVE_PIXELS - 1));
  //     // int step_ms = beat_ms / ACTIVE_PIXELS;
			
	// 		// int64_t now = k_uptime_get();
	// 		// if ((now - last_update) >= step_ms) {
	// 			update_strip(position, cmd.amplitude, colors[cmd.color], cmd.brightness);
	// 			k_sleep(K_USEC(step_ms * 1000));
  //       int whole = (int)step_ms;
  //       int frac  = (int)((step_ms - whole) * 1000); // 3 decimal places
  //       printk("Step_ms = %d.%03d\n", whole, frac);
	// 			position += direction;
	// 			if (position >= ACTIVE_PIXELS - 1) {
	// 				position = ACTIVE_PIXELS - 1;
	// 				direction = -1;
	// 			} else if (position <= 0) {
	// 				position = 0;
	// 				direction = 1;
	// 			}
				
	// 			// last_update = now;
	// 		}
	// 	}
		
	// 	k_sleep(K_MSEC(10));
  //   // k_sleep(K_MSEC(step_ms));
  //   // printk("Step_ms = %d\n", step_ms);
	// }


//   int64_t next_update = k_uptime_get();

// while (1) {
//     // … get cmd …
//     if (k_msgq_get(&lightbar_queue, &cmd, K_NO_WAIT) == 0) {
//     if (cmd.color == 0) {
//       clear_strip();
//       k_msgq_get(&lightbar_queue, &cmd, K_FOREVER);
//     }
// 		} else {
// 			cmd = current_cmd;
// 		}

//     float beat_ms = 60000.0f / cmd.frequency;
//     float step_ms = beat_ms / ACTIVE_PIXELS;

//     if (cmd.color > 0 && cmd.color <= 8) {
//         update_strip(position, cmd.amplitude, colors[cmd.color], cmd.brightness);

//         // schedule next update relative to absolute time
//         next_update += (int64_t)(step_ms);

//         int64_t now = k_uptime_get();
//         int64_t delay = next_update - now;
//         if (delay > 0) {
//             k_sleep(K_USEC(delay*1000));
//         } else {
//             // we're behind schedule → skip ahead
//             next_update = now;
//         }

//         position += direction;
//         if (position >= ACTIVE_PIXELS - 1) {
//             position = ACTIVE_PIXELS - 1;
//             direction = -1;
//         } else if (position <= 0) {
//             position = 0;
//             direction = 1;
//         }
//     }
// }
// }

K_TIMER_DEFINE(step_timer, NULL, NULL);

static void lightbar_thread(void)
{
    struct lightbar_cmd cmd;
    int position = 44;
    int direction = 1;

    while (1) {
        // Get latest command (non-blocking) or keep current
        if (k_msgq_get(&lightbar_queue, &cmd, K_NO_WAIT) == 0) {
            if (cmd.color == 0) {
                clear_strip();
                position = 44;
                k_msgq_get(&lightbar_queue, &cmd, K_FOREVER);
            }
            current_cmd = cmd;
        } else {
            cmd = current_cmd;
        }

        if (cmd.color > 0 && cmd.color <= 8) {
            // calculate timing from BPM
            float beat_ms = 61000.0f / cmd.frequency;
            float step_ms = beat_ms / ACTIVE_PIXELS; 
            int step_us   = (int)(step_ms * 1000);
            printk ("%d", step_us);

            // start / restart timer with this interval
            k_timer_start(&step_timer, K_USEC(step_us), K_USEC(step_us));

            // do one step, then wait for timer for the next
            update_strip(position, cmd.amplitude, colors[cmd.color], cmd.brightness);

            int whole = (int)step_ms;
            int frac  = (int)((step_ms - whole) * 1000); // 3 decimal places
            printk("Step_ms = %d.%03d\n", whole, frac);

            position += direction;
            if (position >= ACTIVE_PIXELS - 1) {
                position = ACTIVE_PIXELS - 1;
                direction = -1;
            } else if (position <= 0) {
                position = 0;
                direction = 1;
            }

            // wait for timer before next update
            k_timer_status_sync(&step_timer);
        } else {
            // if no valid color, idle a bit
            k_sleep(K_MSEC(10));
        }
    }
}


int main(void)
{
	int err;

	printk("Starting Light Bar BLE Peripheral\n");

	if (!gpio_is_ready_dt(&led)) {
		printk("LED GPIO not ready\n");
		return 0;
	}

	if (!device_is_ready(led.port)) {
		printk("LED device not ready\n");
		return 0;
	}

	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
	if (err) {
		printk("Failed to configure LED GPIO\n");
		return 0;
	}

	if (!device_is_ready(strip)) {
		printk("LED strip device not ready\n");
		return 0;
	}

	clear_strip();
	startup_animation();   // run fun boot-up animation

	err = bt_conn_auth_cb_register(&auth_callbacks);
	if (err) {
		printk("Failed to register authorization callbacks.\n");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	k_work_init(&adv_work, adv_work_handler);
	k_work_init_delayable(&blink_work, blink_handler);

	advertising_start();
	blink_start();
	gpio_pin_set_dt(&led, 0);

	printk("Light Bar ready\n");

	return 0;
}

K_THREAD_DEFINE(lightbar_thread_id, STACKSIZE, lightbar_thread,
		NULL, NULL, NULL, PRIORITY, 0, 0);
