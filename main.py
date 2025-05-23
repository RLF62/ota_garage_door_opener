"""

Garage Door Controller code
Adapted from examples in: https://datasheets.raspberrypi.com/picow/connecting-to-the-internet-with-pico-w.pdf

"""
import os
import json
import time
import ntptime
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
from WIFI_CONFIG import ssid, password, static_ip, subnet_mask, gateway_ip, dns_server, garage_name


TEXT_URL = "http://192.168.50.66/pico_ping/ping.html"

# get the current version (stored in version.json)
if 'version.json' in os.listdir():    
    with open('version.json') as f:
        current_version = int(json.load(f)['version'])
    print(f"Current device firmware version is '{current_version}'")


i2c = I2C(id=0, scl=Pin(9), sda=Pin(8), freq=10000)
devices = i2c.scan()

if len(devices) == 0:
    print("No i2c device !")
    distance = 0
    distSensor_available = False
    bme_available = False
else:
    print('i2c devices found:',len(devices))
    bme = BME280.BME280(i2c=i2c, addr=0x77)
    distSensor = PiicoDev_VL53L1X()
    distSensor_available = True
    bme_available = True
    
irq = 0
interrupt_flag = 0
debounce_time = 0
irq_interrupt_flag = 0

def handle_interrupt(irq):
    global debounce_time,interrupt_flag,irq_interrupt_flag
    if (time.ticks_ms()-debounce_time) > 150:
        interrupt_flag = 1
        debounce_time=time.ticks_ms()
        irq_interrupt_flag = irq
        #pin_control_door(irq)
        
int1 = Pin(10, Pin.IN,Pin.PULL_UP) #light
int1.irq(trigger=Pin.IRQ_FALLING, handler=lambda a:handle_interrupt(10))

int2 = Pin(11, Pin.IN,Pin.PULL_UP) #up
int2.irq(trigger=Pin.IRQ_FALLING, handler=lambda a:handle_interrupt(11))

int3 = Pin(12, Pin.IN,Pin.PULL_UP) #down
int3.irq(trigger=Pin.IRQ_FALLING, handler=lambda a:handle_interrupt(12))

int4 = Pin(13, Pin.IN,Pin.PULL_UP) #Vent
int4.irq(trigger=Pin.IRQ_FALLING, handler=lambda a:handle_interrupt(13))


# Hardware definitions
led = Pin("LED", Pin.OUT, value=1)
pin_door = Pin(18, Pin.OUT, value=0)
pin_relay = Pin(20, Pin.OUT, value=0)
pin_light = Pin(22, Pin.OUT, value=0)
adcpin = 4
sensor = machine.ADC(adcpin)
rtc = machine.RTC()

door_distance=0
vent_val=80
#fifty_percent_val=48
#readings=[]
MAX_TIMEOUT=30
current_position=0
button_hold_time=1.5
current_string="Current position is "
check_interval_sec=0.25
garage_status = ""


wlan = network.WLAN(network.STA_IF)

# The following HTML defines the webpage that is served http-equiv="refresh" content="1"   <p>Distance %s inches<p>
#<br><br>
#<center> <button class="button" name="DOOR" value="UD" type="submit">UPDATE FIRMWARE</button></center>
#</center>
#<button class="button" name="DOOR" value="LIGHT" type="submit">LIGHT</button><br><br>
html = """<!DOCTYPE html><html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="viewport2" http-equiv="refresh" content="3, url=/">
<link rel="icon" href="data:,">
<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
.button { background-color: #4CAF50; border: 2px solid #000000; color: white; padding: 15px 32px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; }
.container { background-color: #4CAF50; border: 2px solid #000000; color: white; padding: 15px 32px; text-align: center; text-decoration: none; display: inline-block; font-size: 20px; margin: 4px 2px; cursor: pointer; }
</style></head>
<body> 
<center><h1>""" + garage_name +  """</h1></center><br><br>
<form><center>
<button class="button" name="DOOR" value="UP" type="submit">Open</button><br><br>
<button class="button" name="DOOR" value="DOWN" type="submit">Close</button><br><br>

<button class="button" name="DOOR" value="VENT" type="submit">VENT</button>
</center>
</form>
<br><br>
<div class="container">
<p>%s<p>
</div>
</body></html>
"""

        
def ReadTemperature():
    if bme_available:
        try:
            adc_value = sensor.read_u16()
            volt = (3.3/65535) * adc_value
            temperature = 27 - (volt - 0.706) / 0.001721
            return round(temperature, 1)
        except RuntimeError:
            print("BME sensor not detected!")
            return 0
    else:
        return 0
    
    
def celsius_to_fahrenheit(temp_celsius): 
    temp_fahrenheit = temp_celsius * (9/5) + 32 
    return round(temp_fahrenheit,1)

#def average_of_list(readings):
  #if not readings:
    #return 0  # Return 0 if the list is empty
    #return sum(readings) / len(readings)

def VL53L1X():
    if distSensor_available:    
        try:
            distance = distSensor.read()
            distance = distance * .0393701
            #print(distance)
            return distance
        
        except RuntimeError:
            print("Distance sensor not detected!")
            return 0       
    else:
        return 0

def blink_led(frequency = 0.5, num_blinks = 3):
    for _ in range(num_blinks):
        led.on()
        time.sleep(frequency)
        led.off()
        time.sleep(frequency)

def control_door(cmd):
    pin_relay.on()
    
    if cmd == 'stop':
        pin_door.on()
        led.on()
        time.sleep(button_hold_time)
        led.off()
        pin_door.off()

    if cmd == 'up':
        pin_door.on()
        led.on()
        time.sleep(button_hold_time)
        led.off()
        pin_door.off()

    if cmd == 'down':
        pin_door.on()
        led.on()
        time.sleep(button_hold_time)
        led.off()
        pin_door.off()

    if cmd == 'light':
        pin_light.on()
        led.on()
        time.sleep(button_hold_time)
        led.off()
        pin_light.off()
        
    pin_relay.off()
      
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
    wlan.ifconfig((static_ip, subnet_mask, gateway_ip, dns_server))
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
        ntptime.settime()
        #print(time.localtime())
     
            
            
async def serve_client(reader, writer):
    print("Client connected")
    request_line = await reader.readline()
    print("Request:", request_line)
    # We are not interested in HTTP request headers, skip them
    while await reader.readline() != b"\r\n":
        pass
    
    # find() valid garage-door commands within the request
    request = str(request_line)
    cmd_up = request.find('DOOR=UP')
    cmd_down = request.find('DOOR=DOWN')
    cmd_10 = request.find('DOOR=VENT')
    cmd_light = request.find('DOOR=LIGHT')
    #cmd_firmware = request.find('DOOR=UD')
    
    # Carry out a command if it is found (found at index: 8)
    if bme_available:
        current_position = VL53L1X()
        if current_position <= 40:
            garage_status = "Open"
        elif current_position >= 85:
            garage_status = "Closed"
        elif current_position >= 40 and current_position <= 85:    
            garage_status = "Vented"
            
        #print(time.localtime())
        #dstadjust = 0

        local_date = str(time.localtime()[1]) + "/" + str(time.localtime()[2]) + "/" + str(time.localtime()[0])
        if time.localtime()[3] > 12:
            local_hour = time.localtime()[3] - 12
            local_am_pm = 'PM'
        else:
            local_am_pm = 'AM'
            local_hour = time.localtime()[3]
            

        local_time = str(local_hour) + ":" + str("%02d" % (time.localtime()[4]),) + " " + local_am_pm
        print(current_string + str(current_position))
        temperatureC = ReadTemperature()
        temperatureF = celsius_to_fahrenheit(temperatureC)
        hum = bme.humidity
        hum = float(hum[:-1])
        hum = f"{hum:.1f}"
        tempF = (bme.read_temperature()/100) * (9/5) + 32
        tempF = 'Temp ' + str(round(tempF, 1)) + ' &deg;F<br>'
        tempF = tempF + 'Humidity ' + str(hum) + ' %<br>' + '<br>Door is: ' + garage_status +  '<br>Version: ' + str(current_version) + '<br>'
        tempF = tempF + '<br>' + str(local_time) + '<br>' + str(local_date)
        response = html % tempF
    else:    
        response = html      #temperatureF
    
    
    writer.write('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    writer.write(response)
    if bme_available:
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
    global irq_interrupt_flag
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
        if irq_interrupt_flag != 0:
            pin_control_door(irq_interrupt_flag)
            print(irq_interrupt_flag)
            irq_interrupt_flag = 0
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

