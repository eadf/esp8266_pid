# esp8266-pid
github.com/br3ttb/Arduino-PID-Library ported to esp8266. WORK IN PROGRESS

## Required
esp_iot_sdk_v0.9.4_14_12_19

I have not tested this with v0.9.5. I tested a [clean sdk 0.9.5 install](https://github.com/pfalcon/esp-open-sdk) with one of the basic examples ([blinky](https://github.com/esp8266/source-code-examples)). It compiled and uploaded fine but the esp had a infinite crash loop with some message about "MEM CHK FAIL" on the console. So i threw the whole sdk out (aint nobody got time fo dat). I will try upgrading the sdk again once [mqtt](https://github.com/tuanpmt/esp_mqtt) upgrades to 0.9.5+.
