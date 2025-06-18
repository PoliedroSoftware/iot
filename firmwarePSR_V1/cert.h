/*
En este archivo se encuentran los certificados para realizar la conexión segura con AWS

Estos se descargan directamente del AWS IoT Core cuando se está creando una nueva política
y un nuevo certificado de seguridad en AWS IoT Core. Luego de creado no se pueden descargar.
*/

#ifndef __CERT_H__
#define __CERT_H__

#include <Arduino.h>

const char clientCert[] PROGMEM = "-----BEGIN CERTIFICATE-----\n"
                                  "MIIDWTCCAkGgAwIBAgIUduXNaMHaeXrPo+KxPKwGDFKxnMgwDQYJKoZIhvcNAQEL\n"
                                  "BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g\n"
                                  "SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI1MDMxNDIxMDYx\n"
                                  "OVoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0\n"
                                  "ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAL4lN0gpaFZozLm7843M\n"
                                  "pW6t24KKDa7PhQiMJfjC8GtJz5ZKM9BQJrjvLQg9peEuYBcXwuXHhdxevvrx6IW5\n"
                                  "3X/osuwE2APqX6biX988yzeHUth6qKuilJY26keNOuJZTqHMaFTfMUcmemTUajix\n"
                                  "4kD5/Y+Oi//e9kL1pLxgA1KPlFdSnr4tM5f8UctlBE+MvXkpgGGsO8glxr7NcnPs\n"
                                  "/q1jooeaUDC2ppyXxKYrGZ5YwYtYDzzC5YnLsrnwua1M8XcSMD1w495eRnM+Z0tI\n"
                                  "WsXUHXsej+urYKPNMOk82HywgWlg18ox1ZSAFNtO6fyELDVIylrwnVnYvXY1Qvxr\n"
                                  "VCECAwEAAaNgMF4wHwYDVR0jBBgwFoAUUy30RghAoEj+Kbgn4E0Vw1xsmj8wHQYD\n"
                                  "VR0OBBYEFFj1pyf04kHeJujC768jzMHeatKhMAwGA1UdEwEB/wQCMAAwDgYDVR0P\n"
                                  "AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQAGzlV34tQd5RbvmNs9VjTc7G+h\n"
                                  "KdziA9worwnqbOX9cTPJ33AkV4FzNGDsBxC9+vBaP1w/SNfFkGrB96jNybY1X53/\n"
                                  "Y1AUKzvU15B25ser4ILPoE8d0wDPPvl/1LmRcWkEHOP5VYYWZlA37CAX6ZCJzW8C\n"
                                  "eBuduadVYu+bJmkDhjsDPQvWmKvhjwnUy7y+BTWUhuxyJ+2X0r++fWvqwEvYqyK1\n"
                                  "f4GnPTi9LYejmMi5UnOq9NAjNB7C6W051/kohV4JTACfJbWy9i53JSMPb5wMpGlX\n"
                                  "JBrxqzh9BdhnPmWKtQ2LRQenNhLKMSLsAsvkzzdzdyRDtXFO60DvoU112C/R\n"
                                  "-----END CERTIFICATE-----\n";

const char clientKey[] PROGMEM =  "-----BEGIN RSA PRIVATE KEY-----\n"
                                  "MIIEogIBAAKCAQEAviU3SCloVmjMubvzjcylbq3bgooNrs+FCIwl+MLwa0nPlkoz\n"
                                  "0FAmuO8tCD2l4S5gFxfC5ceF3F6++vHohbndf+iy7ATYA+pfpuJf3zzLN4dS2Hqo\n"
                                  "q6KUljbqR4064llOocxoVN8xRyZ6ZNRqOLHiQPn9j46L/972QvWkvGADUo+UV1Ke\n"
                                  "vi0zl/xRy2UET4y9eSmAYaw7yCXGvs1yc+z+rWOih5pQMLamnJfEpisZnljBi1gP\n"
                                  "PMLlicuyufC5rUzxdxIwPXDj3l5Gcz5nS0haxdQdex6P66tgo80w6TzYfLCBaWDX\n"
                                  "yjHVlIAU207p/IQsNUjKWvCdWdi9djVC/GtUIQIDAQABAoIBAE8ZbkDfsN7ZY+kU\n"
                                  "DgJ8EonTW1CXm4+QFnhyzM0KUzdekjOkFJ/g5+rg3GvkMWBWiXSx1sr90GrXIz2D\n"
                                  "asZOhp1SMTRL44zlb/sdiuJ0CNnFuqoBhK85x/QhTh6L2uYUXsiiHvjHjFlb4WiW\n"
                                  "kMWbAlMja97PXuAHYYyDYY9XpaovBe+/h0SxvC7SLZA/UtNphNddLHriDOCGmXL1\n"
                                  "Dcb/8Scsxu5SwNEzwcKkqjtbu3UDy5gwrx8dsK0Vf8e0ZT3WSZX/6+CCOiz8GIfG\n"
                                  "d5XyqROSa9uejWInXTAIdUIKTxBq6di4yAc9plRdq0VwzI3yUgC+0qZVrlrzVo3k\n"
                                  "i7bAle0CgYEA9HmIOrga9/sxGU7Ua/bfPdwXPzWHxE6qsMjVTzbIL9+2wmco8Wuy\n"
                                  "M9z0GE3vt/d34UnQf2P1zQjElUMIqlg59Zx5/SaTAsJlBQH2Dl0qDHb8Gdi9YNOm\n"
                                  "fzBnre3b0wx+q5Udhcq6uVEWqNtcslIvZKz+VtkTTN+sPTnIy0RmXosCgYEAxxwB\n"
                                  "JMx0PNP079dnp2kUL3FVHOrDoTUuyGJ5wCLYULj6+KSjciHZ389TeWm51EJnI1tv\n"
                                  "GRV2YYEN58U8gTsksuQJP0OeWXWId6ItK1Z1OqI7W0VbuAzKfYcjPlrjj21XHtvV\n"
                                  "tob3WpgXSnK/chepCVis+hfISwQFqyCg2U50OYMCgYAM7eiAgoAaoVRFbF/bneDM\n"
                                  "mrk4BJK7lRV96AEwAUuxDHnT6jTGH9X63DKknmqRJitW5Np8AkNbpp0O786jg6r9\n"
                                  "DFt/Qe0Adlt1WlczB0ZUBm7qxwWs+0SQ8XRqRGwdy0lftbEsBD6/0kQKhh9u5t3O\n"
                                  "yz9gRHa62BF543Z1GZb4ZQKBgGNNDzD4GtyfZSAc0OBIYr3KiwVR/GEw8fUUT95I\n"
                                  "QpP3vYs2KSHpygx6DQqXdQtsVp3mowDimFjGkVXPI37cLVBetFHt+lJpcLchld9w\n"
                                  "TiUA9hNuAvFGx9JVDHmzwJwuju2f+/T9WO3AvpTtp5dSP4aeyB4usEDhb6ZKLo5S\n"
                                  "N1rbAoGAUonXGjGGAHjNulVWUt+7ZT+zWS6hImVa2gYiVz8UJz9ZjgVH8VC6NKDV\n"
                                  "5V+ARrcUukrofCq9dizivjpxUVl66uDXBnaRbSp5YdZUsbf17gFrTQHCspBBYXhl\n"
                                  "Kpf8Ycjome3ShMHbD107Vld8xlkr8CIu71kdtLQkvT9506fq9do=\n"
                                  "-----END RSA PRIVATE KEY-----\n";

const char rootCACert[] PROGMEM = "-----BEGIN CERTIFICATE-----\n"
                                  "MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n"
                                  "ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n"
                                  "b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n"
                                  "MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n"
                                  "b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n"
                                  "ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n"
                                  "9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n"
                                  "IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n"
                                  "VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n"
                                  "93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n"
                                  "jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n"
                                  "AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n"
                                  "A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n"
                                  "U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n"
                                  "N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n"
                                  "o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n"
                                  "5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n"
                                  "rqXRfboQnoZsG4q5WTP468SQvvG5\n"
                                  "-----END CERTIFICATE-----\n";

#endif