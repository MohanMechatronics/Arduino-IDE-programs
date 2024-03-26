#include <pgmspace.h>

#define SECRET
#define THINGNAME "mohan"

const char WIFI_SSID[] = "Iam Disguised";
const char WIFI_PASSWORD[] = "gunasree";
const char AWS_IOT_ENDPOINT[] = "a2al39a3lbbq6b-ats.iot.ap-northeast-1.amazonaws.com";

// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

// Device Certificate
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVANQd7T5K262zNCJ8HTqzMyAu/+H0MA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yMzAyMjAxNjIx
MzVaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDf0i67alshaLajsSyj
VEyRZtcZqEnfzHzw7pC/jh6y7xTRP4vsYHHHRqQkYJtu70zfZDFE7mkvBeI+VEk9
nUwTyLNqkPi4zKpXeBtIp91xfwQ42m0CddrsD/lG8AsLQU2o84yyhPgvQcsJjYk4
Scp/LslC6PkkVLTY9JNJhh583hK15PJU5xb63sjDVZsqi6eI4GwUPfQL1qtOck6K
BxzziIqFGBsWxfqiUhOKIq72sfbIc/1dg0Uw/4DHB6V+4oCbvYi0TeKis8gjfuJ2
gkf85kntLkjMSZZUBnowDf+u9JL+e/7mlNt/6YdeGlJBjoDzTc2ZEtiModIZXIY+
n4BNAgMBAAGjYDBeMB8GA1UdIwQYMBaAFAg0RZb+37wXwR+kKWmomNZwj7pRMB0G
A1UdDgQWBBRATvyd2zUUqEl6LJh55RfHpwOM+TAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAldkF+LUPq7iFBdkYe85wiAQy
amAVdNYr34kWosSNUHK732oJbOTZFwYKfaOIoYXJdflcaguwydjcabKlnyKpDSKT
ga+c1BChzwcNnNHaYS+7mAKizf/gtWK1MnSK9dqaIeuZ/WbdOIV51Doryc1F6KUN
jH4qFVYHhu1YZZHryKqxT37oVXEVtlLBdh8MLSfwaNBHUHGz1vpE1HfnvNCFOwqR
bExEU1HSiBkDVZSyt0iZ9qt3R7GwuQwG+ryP5/wEwF+/VQs/ZRUht+75j6kfwsPR
rbPm3qBYWbws+WvUfPKP9gJEDGJeSFLyXGR3eBjVx6x2hzvjOl08j2RM9gdAVA==
-----END CERTIFICATE-----

)KEY";

// Device Private Key
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEA39Iuu2pbIWi2o7Eso1RMkWbXGahJ38x88O6Qv44esu8U0T+L
7GBxx0akJGCbbu9M32QxRO5pLwXiPlRJPZ1ME8izapD4uMyqV3gbSKfdcX8EONpt
AnXa7A/5RvALC0FNqPOMsoT4L0HLCY2JOEnKfy7JQuj5JFS02PSTSYYefN4SteTy
VOcW+t7Iw1WbKouniOBsFD30C9arTnJOigcc84iKhRgbFsX6olITiiKu9rH2yHP9
XYNFMP+AxwelfuKAm72ItE3iorPII37idoJH/OZJ7S5IzEmWVAZ6MA3/rvSS/nv+
5pTbf+mHXhpSQY6A803NmRLYjKHSGVyGPp+ATQIDAQABAoIBAQC/XdP7Bgp43DEc
cEX8N/1nbIiJNWxEX3CMB03XmkoBL42bAzw9lJBzk6CQQdFj3lLZxa6igz2HqP4x
0Jr2hWEgoT9V5DHkv4j9VIHvcydQ/Qq9GXSO3/mYUGgQEPlAXEtsIyCvGiWhMh3J
QyyJ5f2vFG+cN9mdCJe/0A2GlZfHpfYcrFG3vM7mACOpRllgf1jFra7U/0fmzYAr
eQ+KiaAZSDSJ+ymgyNBhjUOf1eiyVRq8/aaMgLc6Q+BUETTuMSqYH9n9TJSZBpuj
da11SQr4iDlPTbjka1zHckWyeqDaYtiWpksV/Oi9shFVYdVl6fVKwv4PZwkDfcZJ
asCdGpd5AoGBAP+24HB/4aBH7Tln4mAtcoZZLTvs+0WUYFKAoLKGY7085zF6RWVL
ArKIyjo8r6yYcxiGncex5FhtxnX4l2fF2EQiP3BmdlVasVQawxhIn8WtC/hw4klS
as3lsDx7bb8qAl0pXbhnmu8I0Ey93IVeaOZcloJ6S2VabWP7PlrPEON3AoGBAOAS
L4rG++mKyh2jDyWAw3PRyNwF5k7vidUTedv1fQDfhLGsX0C89VbyOTw1eZ9Ki+QM
v0lHQoxCTKqb654dcw5MQG+UEZoHlgbjfjWQzZMEUX84TSqoiIfH5V8sDurC00tY
EfWhKLG6U8IGbYCu68SwrtQa31nt1fqSKCzFPcNbAoGAHZdsIZiq44FvUlVTPQ4F
Usa/FVA3Q7YH74QzFHW41Q1XM8KShUZI6aAqxtJ/Adoid5Qcbh+H7TPG46AA5WFf
xYKxFLDGwNnd632tVgIRey6rU/S8Ov/unCUYX+doq9aQr3c6ATz8EO1ULY1LF6P1
P7M3N6IvV2s/fXJ1pMyRh8ECgYBZ2fEGjVRnCiiSSdjBCSlXQEBzOkpoTNXQg8bR
1VcTCo6DUMPBigQ3oVoiMq9amOIPyKzMDa853zwvDecluFly2lDcbVvBkFkw2fi6
Mh69XXXwtfr7qClXiWkJCPnSWd2w0Ngiz0UqP7VmkH1IpNwWO2Nf2obF4QF1QTaq
/U0PEQKBgDjqU+h9+hhiaMuCvwdaRAEf0V8qO9Vnz/VT1jK+/SxxYxFr3DD9jwVQ
rkNl2u9rAUcf8UlcDEjLOQW9Fh8Ev3URHPBQxFM0zU/Ci3k7e65owUNXI1bb9ade
ASgxYrVqNzaL435BJoyk/ITweJO/6mD9AQNbcocEMnNKJaR9+Rkz
-----END RSA PRIVATE KEY-----

)KEY";