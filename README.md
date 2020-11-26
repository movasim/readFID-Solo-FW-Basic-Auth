# readFID-Solo-FW-Basic-Auth
### Basic-Authentication *(User/Password)* Firmware flavor for readFID Solo.

This is the Basic-Authentication Firmware flavor of the **readFID Solo** device, providing Client-Authentication by means of User-Password defined at compilation time. Its security features are described in the following table.

| Client Authentication | Server Authentication | Data Encryption | Secure Element |
| :-------------------- | :-------------------- | :-------------- | :------------- |
| Yes *(User/Pasword)*  | No                    | No              | No             |

The Firmware reads sensors and device status and reports two types of MQTT messages to **IoTek-RFID Service**, both in JSON format.

The **Sensors Data** message is sent to **IoTek-RFID** as soon as a new RFID Tag is read, contains the following information:

```json
{"msg_type":"srs","tag_id":"0xa61f0e13","tag_type":"MIFARE 1KB","ts":1606379521,"alias":"Production"}
```

Where:

| Key        | Key Description | Value        | Value Description                      |
| ---------- | --------------- | ------------ | -------------------------------------- |
| `Msg_type` | Message Type    | `srs`        | Sensors Data.                          |
| `tag_id`   | Tag ID          | `0xa61f0e13` | ID of the RFID Tag read.               |
| `tag_type` | Tag Type        | `MIFARE 1KB` | See supported Tag Types **here**.      |
| `ts`       | Timestamp       | `1606379521` | Timestamp in UNIX format.              |
| `alias`    | Alias           | `Production` | Friendly name for the reader location. |

And the **Device Data** message contains the following information:

```json
{"msg_type":"dev","dev_t":"readFID","dev_m":"Solo","dev_v":"1.0.0","fw_f":"Basic-Auth","fw_v":"1.0.0","mac":"84:F3:EB:E3:A8:61","ip":"192.168.4.117","s_qty":74,"up":600005,"ts":1606380101,"t":25.25,"rst_r":"Power On","free_heap":45784,"heap_frg":2}
```

Where:

| Key         | Key Description           | Value               | Value Description                                            |
| ----------- | ------------------------- | ------------------- | ------------------------------------------------------------ |
| `Msg_type`  | Message Type              | `dev`               | Device Data.                                                 |
| `dev_t`     | Device Type               | `readFID`           | -                                                            |
| `dev_m`     | Device Model              | `Solo`              | -                                                            |
| `dev_v`     | Device Version            | `1.0.0`             | Hardware Version.                                            |
| `fw_f`      | Firmware Flavor           | `Basic-Auth`        | -                                                            |
| `fw_v`      | Firmware Version          | `1.0.0`             | -                                                            |
| `mac`       | MAC                       | `84:F3:EB:E3:A8:61` | -                                                            |
| `ip`        | IP                        | `192.168.4.117`     | -                                                            |
| `s_qty`     | Signal Quality            | `74`                | Quality of the WiFi signal measured by ESP8266.              |
| `up`        | Uptime                    | `600005`            | Milliseconds since power-up.                                 |
| ts          | Timestamp                 | `1606380101`        | Timestamp in UNIX format.                                    |
| t           | Temperature               | `25.25`             | [ºC] Celsius Degrees.                                        |
| `rst_r`     | Reseat Reason             | `Power On`          | Reason of the last Device Reset.                             |
| `free_heap` | Free Memory Heap          | `45784`             | free heap size in bytes.                                     |
| `heap_frg`  | Memory Heap Fragmentation | `2`                 | [%] Percentage. *(0% is clean, more than ~50% is not harmless).* |

Periodicity of the Device report to **IoTek-RFID Service** can be configured in the *"Device User Parametrization"* section, described in the section below.

## Firmware Configuration and Compilation

This project has been developed in [VSCODE/PlatformIO](VSCODE/PlatformIO), and includes all the necessary libraries for its compilation in the `/lib` directory. That is, by just downloading *(or cloning)* the project and editing one file name, it should compile without errors.

Once in the project's local folder, rename the file that is inside the `/include` directory from `mqtt.configuration.h` to `mqtt.configuration.example.h` *( just delete "example")*.

The content of the mqtt.configuration.example.h file is as follows:

```
// Replace with your credentials

#define MQTT_USER "user"
#define MQTT_PASSWORD "password"
#define MQTT_SERVER_IP 192, 168, 1, 1
#define MQTT_SERVER "mqtt.server.io"
#define MQTT_SERVER_PORT 1883
#define MQTT_PUBLISH_TOPIC  "t"
#define MQTT_SUBSCRIBE_TOPIC "command///req/#"
```

It is neccesary to edit the file contents with the MQTT data that should be provided by **IoTek-RFID Service** when the device is provisioned in the platform.

In summary, the steps to follow to compile and install the readFID Solo Basic-Authentication Firmware on the microcontroller board are the following:

1. Download the project code *(or clone it in the desired directory)*.
2. Rename the file inside the `/include` directory from `mqtt.configuration.example.h` to `mqtt.configuration.h` *(just delete "example")*.
3. Edit the file giving it the MQTT information provided by **IoTek-RFID service** at the moment of provisioning your device.
4. Compile the project.
5. Download it to the readFID Solo board via USB cable.

Only for those cases where is neccesary to modify pre-defined behaviuor of the Firmware, those modifications should be done in the "**Device User Parametrization**" section at the begining of the `src/main.cpp` file, shown below:

```c++
// ========== Start Device User Parametrization ================================================================

#define Alias "Production";                     // Friendly name for the Reader location.
unsigned int mqttDeviceReportPeriod = 600000;    // Device Report Period (Miliseconds).
int resetPortal = 180;                          // Number of seconds until the WiFiManager resests ESP8266.
#define AP_Password "readFID.movasim"           // AP password.

// ========== Start Device Development Parametrization (ONLY MODIFY WHEN NEW HW/FW VERSION IS RELASED) =========

#define DeviceType "readFID" 
#define DeviceModel "Solo"
#define DeviceVersion "1.0.0"
#define FirmwareFlavor "Basic-Auth"
#define FirmwareVersion "1.0.0"
#define JUMPER 10
#define SCL D1
#define SDA D2
#define SS_PIN D4
#define RST_PIN D3
#define BUZZER D8
#define LED D0

// ========== End Device Parametrization =======================================================================

```

instead, the "**Device Development Parametrization**" section should only be modified when a new device hardware or firmware versions are released.

## License

This project by [MOVASIM](https://movasim.com/) is licensed under the [GNU General Public License V3.0](https://github.com/movasim/readFID-Solo-FW-Basic-Auth/blob/main/LICENSE).
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](http://www.gnu.org/licenses/gpl-3.0)

