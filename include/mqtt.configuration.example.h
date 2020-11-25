/*
  readFID Solo. Basic-Authentication Firmware flavor.
  RFID Reader with embedded antenna and WiFi connectivity.

  Copyright (C) 2020  MOVASIM (https://movasim.com/)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  */

// Replace with your credentials

#define MQTT_USER "user"
#define MQTT_PASSWORD "password"
#define MQTT_SERVER_IP 192, 168, 1, 1
#define MQTT_SERVER "mqtt.server.io"
#define MQTT_SERVER_PORT 1883
#define MQTT_PUBLISH_TOPIC  "t"
#define MQTT_SUBSCRIBE_TOPIC "command///req/#"
