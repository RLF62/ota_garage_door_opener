"""

Garage Door Controller code
Adapted from examples in: https://datasheets.raspberrypi.com/picow/connecting-to-the-internet-with-pico-w.pdf

"""
import os
import json
import ujson as json
import time
import utime
import network
import uasyncio as asyncio
import BME280
import machine
import gc
import urequests as requests
from PiicoDev_VL53L1X import PiicoDev_VL53L1X
from machine import Pin, I2C
from ota import OTAUpdater
from WIFI_CONFIG import ssid, password

TEXT_URL = "http://192.168.50.69/pico_ping/ping.html"

# get the current version (stored in version.json)
if 'version.json' in os.listdir():    
    with open('version.json') as f:
        current_version = int(json.load(f)['version'])
    print(f"Current device firmware version is '{current_version}'")


i2c = I2C(id=0, scl=Pin(9), sda=Pin(8), freq=10000)

bme = BME280.BME280(i2c=i2c, addr=0x77)
distSensor = PiicoDev_VL53L1X()

irq = 0
int1 = Pin(10, Pin.IN,Pin.PULL_UP) #light
int1.irq(trigger=Pin.IRQ_FALLING, handler=lambda a:handle_interrupt(10))

int2 = Pin(11, Pin.IN,Pin.PULL_UP) #up
int2.irq(trigger=Pin.IRQ_FALLING, handler=lambda a:handle_interrupt(11))

int3 = Pin(12, Pin.IN,Pin.PULL_UP) #down
int3.irq(trigger=Pin.IRQ_FALLING, handler=lambda a:handle_interrupt(12))

int4 = Pin(13, Pin.IN,Pin.PULL_UP) #10%
int4.irq(trigger=Pin.IRQ_FALLING, handler=lambda a:handle_interrupt(13))
debounce_time=0

# Hardware definitions
led = Pin("LED", Pin.OUT, value=1)
pin_stop = Pin(18, Pin.OUT, value=0)
pin_light = Pin(22, Pin.OUT, value=0)
adcpin = 4
sensor = machine.ADC(adcpin)

door_distance=0
vent_val=80
fifty_percent_val=48
MAX_TIMEOUT=30
readings=[]
current_position=0
button_hold_time=1.5
current_string="Current position is "
check_interval_sec=0.25
garage_status = ""
wlan = network.WLAN(network.STA_IF)


def get_manifest_json():
    manifest = {
        "name": "Garage Door Controller",
        "short_name": "GarageDoor",
        "display": "standalone",
        "theme_color": "#4A90E2",
        "background_color": "#4A90E2",
        "icons": [
            {
                "src": "icon.png",
                "type": "image/png",
                "sizes": "192x192"
            }
        ]
    }
    return json.dumps(manifest)


# The following HTML defines the webpage that is served http-equiv="refresh" content="1"   <p>Distance %s inches<p>
#<br><br>
#<center> <button class="button" name="DOOR" value="UD" type="submit">UPDATE FIRMWARE</button></center>
#</center>

html = """<!DOCTYPE html><html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,">
    <style>
        html {
            font-family: Helvetica;
            display: inline-block;
            margin: 0px auto;
            text-align: center;
        }
        .button, .homeButton {
            background-color: #4CAF50;
            border: none;
            color: white;
            padding: 15px 32px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            margin: 4px 2px;
            cursor: pointer;
        }
        .buttonRed {
            background-color: #d11d53;
            border: none;
            color: white;
            padding: 15px 32px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            margin: 4px 2px;
            cursor: pointer;
        }
        .homeButton {
            background-color: #008cba;
            border: none; /* Different color for home button */
        }
    </style>
    <link rel="manifest" href="/manifest.json">
</head>
<body> 
<center><h1>Garage Door Main</h1></center><br><br>
<form><center>
<button class="button" name="DOOR" value="UP" type="submit">Open</button><br><br>
<button class="button" name="DOOR" value="DOWN" type="submit">Close</button><br><br>
<button class="button" name="DOOR" value="LIGHT" type="submit">LIGHT</button><br><br>
<button class="button" name="DOOR" value="VENT" type="submit">VENT</button>
<a href="http://192.168.50.59:5000" class="homeButton">Home Dashboard</a>  <!-- Home button link -->
</center>
</form>
<br><br>
<br><br>
<p>%s<p>
</body></html>
"""

        
def ReadTemperature():
    adc_value = sensor.read_u16()
    volt = (3.3/65535) * adc_value
    temperature = 27 - (volt - 0.706) / 0.001721
    return round(temperature, 1)

def celsius_to_fahrenheit(temp_celsius): 
    temp_fahrenheit = temp_celsius * (9/5) + 32 
    return round(temp_fahrenheit,1)

def handle_interrupt(irq):
    interrupt_flag, debounce_time, irq_interrupt_flag
    if (time.ticks_ms()-debounce_time) > 10000:
        interrupt_flag = 1
        debounce_time=time.ticks_ms()
        irq_interrupt_flag = 1
        pin_control_door(irq)
 
def average_of_list(readings):
  if not readings:
    return 0  # Return 0 if the list is empty
    return sum(readings) / len(readings)

def VL53L1X():
    distance = distSensor.read()
    distance = distance * .0393701
    #print(distance)
    return distance

def blink_led(frequency = 0.5, num_blinks = 3):
    for _ in range(num_blinks):
        led.on()
        time.sleep(frequency)
        led.off()
        time.sleep(frequency)

def control_door(cmd):
    if cmd == 'stop':
        pin_stop.on()
        led.on()
        time.sleep(button_hold_time)
        led.off()
        pin_stop.off()

    if cmd == 'up':
        pin_stop.on()
        led.on()
        time.sleep(button_hold_time)
        led.off()
        pin_stop.off()

    if cmd == 'down':
        pin_stop.on()
        led.on()
        time.sleep(button_hold_time)
        led.off()
        pin_stop.off()

    if cmd == 'light':
        pin_light.on()
        led.on()
        time.sleep(button_hold_time)
        led.off()
        pin_light.off()
  
def pin_control_door(pin_cmd):
    current_position = VL53L1X()
    print(current_string + str(current_position))
    if pin_cmd == 10:
        control_door('light')
      
    elif pin_cmd == 11:
        control_door('up')
        time.sleep(2)
        if VL53L1X() >= current_position:
            control_door('stop')
            time.sleep(3)
            control_door('up')
        
    elif pin_cmd == 12:
        control_door('down')
        time.sleep(2)
        if VL53L1X() <= current_position:
            control_door('stop')
            time.sleep(3)
            control_door('down')        
        
    elif pin_cmd == 13:
        start_time = time.time()
        if VL53L1X() < vent_val:
            control_door('down')
            time.sleep(2)
            if VL53L1X() < current_position:
                control_door('stop')
                time.sleep(3)
                control_door('down')
            time_delta = time.time() - start_time
            while VL53L1X() <= vent_val and time_delta <= MAX_TIMEOUT:
                time_delta = time.time() - start_time
                led.on()
                print(current_string + str(VL53L1X()) + " of " + str(vent_val))
            else:
                control_door('stop')
                led.off()
        elif VL53L1X() > vent_val:
            time_delta = time.time() - start_time
            control_door('up')
            time.sleep(2)
            if VL53L1X() > current_position:
                control_door('stop')
                time.sleep(3)
                control_door('up')            
            while VL53L1X() >= vent_val and time_delta <= MAX_TIMEOUT:
                time_delta = time.time() - start_time
                led.on()
                print(current_string + str(VL53L1X()) + " of " + str(vent_val))
            else:
                control_door('stop')
                led.off()
                
async def connect_to_wifi():
    wlan.active(True)
    wlan.config(pm = 0xa11140)  # Disable powersave mode
    wlan.connect(ssid, password)

    # Wait for connect or fail
    max_wait = 10
    while max_wait > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        max_wait -= 1
        print('waiting for connection...')
        time.sleep(1)

    # Handle connection error
    if wlan.status() != 3:
        blink_led(0.1, 10)
        raise RuntimeError('WiFi connection failed')
    else:
        blink_led(0.5, 2)
        print('connected')
        status = wlan.ifconfig()
        print('ip = ' + status[0])


async def serve_client(reader, writer):
    print("Client connected")
    request_line = await reader.readline()
    print("Request:", request_line)
    # We are not interested in HTTP request headers, skip them
    while await reader.readline() != b"\r\n":
        pass
    
    # find() valid garage-door commands within the request
    request = str(request_line)
    if '/manifest.json' in request:
        manifest_data = get_manifest_json()
        writer.write('HTTP/1.0 200 OK\r\nContent-type: application/manifest+json\r\n\r\n')
        writer.write(manifest_data.encode())
    elif '/icon.png' in request:
        try:
            with open('icon.png', 'rb') as f:
                icon_data = f.read()
            writer.write(b'HTTP/1.0 200 OK\r\nContent-Type: image/png\r\n\r\n' + icon_data)
        except FileNotFoundError:
            writer.write(b'HTTP/1.0 404 Not Found\r\n\r\n')
    else:
    
    cmd_up = request.find('DOOR=UP')
    cmd_down = request.find('DOOR=DOWN')
    cmd_10 = request.find('DOOR=VENT')
    cmd_light = request.find('DOOR=LIGHT')
    #cmd_firmware = request.find('DOOR=UD')
    
    # Carry out a command if it is found (found at index: 8)
    current_position = VL53L1X()
    if current_position <= 40:
        garage_status = "Open"
    elif current_position >= 85:
        garage_status = "Closed"
    elif current_position >= 40 and current_position <= 85:    
        garage_status = "Vented"
        
        
    print(current_string + str(current_position))
    temperatureC = ReadTemperature()
    temperatureF = celsius_to_fahrenheit(temperatureC)
    hum = bme.humidity
    tempF = (bme.read_temperature()/100) * (9/5) + 32
    tempF = 'Temp ' + str(round(tempF, 2)) + '&deg;F<br>'
    tempF = tempF + 'Humidity ' + hum + '<br>Door is: ' + garage_status +  '<br>Version: ' + str(current_version) 

    response = html % tempF      #temperatureF
    writer.write('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    writer.write(response)
    
    if cmd_up == 8:
        control_door('up')
        time.sleep(2)
        if VL53L1X() >= current_position:
            control_door('stop')
            time.sleep(3)
            control_door('up')
    elif cmd_down == 8:
        control_door('down')
        time.sleep(2)
        if VL53L1X() <= current_position:
            control_door('stop')
            time.sleep(3)
            control_door('down')        
    elif cmd_10 == 8:
        start_time = time.time()
        if VL53L1X() < vent_val:
            control_door('down')
            time.sleep(2)
            if VL53L1X() < current_position:
                control_door('stop')
                time.sleep(3)
                control_door('down')
            time_delta = time.time() - start_time
            while VL53L1X() <= vent_val and time_delta <= MAX_TIMEOUT:
                time_delta = time.time() - start_time
                led.on()
                print(current_string + str(VL53L1X()) + " of " + str(vent_val))
            else:
                control_door('stop')
                led.off()
        elif VL53L1X() > vent_val:
            time_delta = time.time() - start_time
            control_door('up')
            time.sleep(2)
            if VL53L1X() > current_position:
                control_door('stop')
                time.sleep(3)
                control_door('up')            
            while VL53L1X() >= vent_val and time_delta <= MAX_TIMEOUT:
                time_delta = time.time() - start_time
                led.on()
                print(current_string + str(VL53L1X()) + " of " + str(vent_val))
            else:
                control_door('stop')
                led.off()
    elif cmd_light == 8:
        control_door('light')
    #elif cmd_firmware == 8:
        #firmware_url = "https://raw.githubusercontent.com/RLF62/ota_garage_door_opener/"
        #ota_updater = OTAUpdater(ssid,password,firmware_url,"main.py")
        #ota_updater.download_and_install_update_if_available()

    await writer.drain()
    await writer.wait_closed()
    

async def main():
    print('Connecting to WiFi...')
    asyncio.create_task(connect_to_wifi())
    #print('Setting up webserver...')
    asyncio.create_task(asyncio.start_server(serve_client, "0.0.0.0", 80))
    firmware_url = "https://raw.githubusercontent.com/RLF62/ota_garage_door_opener/"
    ota_updater = OTAUpdater(ssid,password,firmware_url,"main.py")
    ota_updater.download_and_install_update_if_available() 
    interval_sec = 0
    increment_sec = 30
    timer_sec = time.time() - interval_sec
    
    while True:
        gc.collect()
        if time.time() - timer_sec > interval_sec:
            try:
                r = requests.get(TEXT_URL)
                print(f"{interval_sec:>5} {r.status_code} {r.reason.decode()} {r.content}")
                r.close()
                interval_sec += increment_sec
            except OSError as e:
                print(e)
                machine.reset()
            timer_sec = time.time()

        await asyncio.sleep(check_interval_sec)

try:
    asyncio.run(main())

finally:
    asyncio.new_event_loop()
