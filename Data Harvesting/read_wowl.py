import asyncio
from datetime import datetime
import time
import numpy as np
import pandas as pd
from bleak import BleakScanner,BleakClient
from bleak.exc import BleakError
import sqlite3
from sqlite3 import Error

BASE_UUID = "-0000-1000-8000-00805f9b34fb"
WOWL_SERVICE_UUID = "00003090" + BASE_UUID
tmp_byte_array,recieved_records,num_downloaded,conn=[],[],0,None #these will be global variables

def COMMAND(c,BASE_UUID=BASE_UUID):
    """
    Runs full length WOWL commands using only the 4 digit code.
    """
    return "0000"+str(c)+BASE_UUID  

def addData(data):
    """
    Sends WOWL data to global variables.
    """
    global tmp_byte_array
    global recieved_records
    global num_downloaded
    if len(tmp_byte_array)<4*20:
        tmp_byte_array.extend(data)
        if len(tmp_byte_array)==4*20:
            recieved_records.append(parseBytes(tmp_byte_array[4:8],type="u32"))
            df = parseWOWLData(tmp_byte_array[0:68])
            saveData(df)
            tmp_byte_array=[]

def notificationHandler(sender, data):
    """
    Recieves data from the WOWL and runs addData.
    """
    addData(data)

def checkForWowl(devices):
    """
    Checks BLE devices for WOWLs, detected if they contain WOWL_SERVICE_UUID.
    """
    WOWLS=[]
    for d in devices:
        if WOWL_SERVICE_UUID in d.metadata["uuids"]:
            WOWLS.append(d.address)
    print("Found "+str(len(WOWLS))+" WOWLS: "+str(WOWLS))
    return WOWLS

def progressBar(current, total, barLength = 20):
    """
    Track progress of downloading WOWL data.
    """
    percent = float(current) * 100 / total
    arrow   = '-' * int(percent/100 * barLength - 1) + '>'
    spaces  = ' ' * (barLength - len(arrow))
    print('Progress: [%s%s] %d %% %s %s %s' % (arrow, spaces, percent, "   ("+str(current), "/", str(total)+") Records"), end='\r')

async def downloadData(WOWL,num_requested=16,delete_records=False,update_firmware=False,reset=False):
    """
    TODO: While downloading data make sure the indices match up?

    Function to download data from the WOWL
    1) Connects to WOWL
    2) Requests the total number of records (code=3092)
    3) Requests notifications for download (code=3095)
    4) Requests num_requested records be transmitted (code=3094)
    5) Tells WOWL to begin transmitting data (code=3096:1)
    6) Repeats 3-5 until all records are downloaded
    """
    # global recieved_records
    global num_downloaded
    global tmp_byte_array
    global recieved_records
    num_downloaded=0
    tmp_byte_array=[]
    recieved_records=[]
    device = await BleakScanner.find_device_by_address(WOWL, timeout=5.0)
    if not device:
        print("No device found")
        raise BleakError(f"A WOWL with address {WOWL} could not be found.")
    print("Connected to WOWL @ "+str(WOWL))
    connectTime=datetime.now()
    currentLocation="coming soon"
    async with BleakClient(device) as client:
        #reset for development purposes
        if reset==True:
            print("RESETTING")
            await client.write_gatt_char(COMMAND(3096), bytearray([3])) #for resetting
        try:
            num_records_raw = await client.read_gatt_char(COMMAND(3092))
            num_records = num_records_raw[0] | num_records_raw[1] << 8
            print("Number of Records: "+str(num_records))
            if num_records>0:
                print("Number of records:",num_records,"& Records per Request",num_requested)
                await client.start_notify(COMMAND(3095), notificationHandler)
                offset=0-num_requested
                """
                When checking if records are missing, need to account for num_requested, they can come in any order
                """
                while offset<num_records:
                    #check if we missed a record
                    if offset+num_requested>num_downloaded:
                        for i in range(0,len(recieved_records)):
                            if i!=recieved_records[i]:
                                offset+=i
                                num_requested=1
                    else:
                        offset+=num_requested
                        #do not request more records than on the WOWL
                        if num_records < offset + num_requested:
                            num_requested=num_records-offset
                    
                    await client.write_gatt_char(COMMAND(3094), bytearray(np.array([offset,num_requested]).astype(np.uint16)))
                    await client.write_gatt_char(COMMAND(3096), bytearray([1]))
                    downloading,timeout_counter,sleep_time,timeout_time=True,0,0.25,3.0
                    while downloading:
                        try:
                            progressBar(num_downloaded, num_records, barLength = 20)
                            #break out of while loop after data has been downloaded
                            if num_downloaded==offset+num_requested:
                                downloading=False
                                timeout_counter=0
                            else:
                                await asyncio.sleep(sleep_time)
                                if timeout_counter*sleep_time>=timeout_time:
                                    print("Timeout after waiting too long for data")
                                    print("Requesting data again...")
                                    downloading=False
                                timeout_counter+=1

                        except asyncio.CancelledError:
                            print("Shutdown Request Received")
                            break
                    
                if delete_records==True:
                    print("Currently disabled to not accidently delete data")
                    # await client.write_gatt_char(COMMAND(3096), bytearray([2]))
                    # print("Data Erased")

                if update_firmware==True:
                    print("Feature under development")

                await client.stop_notify(COMMAND(3095))
                print("")
                print("Records Logged:",num_downloaded)

        except:
            print("Issue with connected device")

        finally:
            # await client.write_gatt_char(COMMAND(3096), bytearray([3]))
            await client.disconnect()

def parseBytes(bytes,type="u16"):
    """
    Parses bytes based on specified type.
    """
    if type=="u16":
        return bytes[0] << 0 | bytes[1] << 8
    if type=="u32":
        return bytes[0] << 0 | bytes[1] << 8 | bytes[2] << 16 | bytes[3] << 24
    if type=="f32":
        return np.array(bytes, dtype=np.uint8).view(dtype=np.float32)[0]

def parseWOWLData(d):
    """
    Convert recieved data into dataframe.
    """
    columns = ["recordIndex","recordStatus","measurementCount","status","reserved",
    "chargingVoltage","chargingCurrent","rectifiedChargingCoilVoltage","batteryVoltage",
    "CPUTemperature","temperatureNTC","temperatureRTD","temperatureSI7051","pressure",
    "temperatureFromPressure","temperatureNTCADC","temperatureRTDADC","CRC32"]
    parsedData = pd.DataFrame(columns=columns)

    recordIndex=parseBytes(d[0:2],type="u16")
    recordStatus=parseBytes(d[2:4],type="u16")
    measurementCount=parseBytes(d[4:8],type="u32")
    status=parseBytes(d[8:12],type="u32")
    reserved=parseBytes(d[12:16],type="u32")
    chargingVoltage=parseBytes(d[16:20],type="f32")
    chargingCurrent=parseBytes(d[20:24],type="f32")
    rectifiedChargingCoilVoltage=parseBytes(d[24:28],type="f32")
    batteryVoltage=parseBytes(d[28:32],type="f32")
    CPUTemperature=parseBytes(d[32:36],type="f32")
    temperatureNTC=parseBytes(d[36:40],type="f32")
    temperatureRTD=parseBytes(d[40:44],type="f32")
    temperatureSI7051=parseBytes(d[44:48],type="f32")
    pressure=parseBytes(d[48:52],type="f32")
    temperatureFromPressure=parseBytes(d[52:56],type="f32")
    temperatureNTCADC=parseBytes(d[56:60],type="f32")
    temperatureRTDADC=parseBytes(d[60:64],type="f32")
    CRC32=parseBytes(d[64:68],type="u32")

    data = [recordIndex,recordStatus,measurementCount,status,reserved,
    chargingVoltage,chargingCurrent,rectifiedChargingCoilVoltage,batteryVoltage,
    CPUTemperature,temperatureNTC,temperatureRTD,temperatureSI7051,pressure,
    temperatureFromPressure,temperatureNTCADC,temperatureRTDADC,CRC32]

    """
    Need to add metadata, location, etc. columns. These will be populated
    when saving row of data to db.
    """
    return pd.DataFrame(data,columns).T

def saveData(df):
    """
    Saves the df row to db.
    """
    global conn
    global num_downloaded
    print("Saved to db, data will be sent to server eventually")
    # file_name = "example_data.csv"
    # df.to_csv(file_name, sep=',', encoding='utf-8')
    """
    Before appending to sql, add metadata, location, etc.
    """
    d = {'Field1': num_downloaded}
    df_tester = pd.DataFrame(data=d)
    df_tester.to_sql("WOWL_Records",conn,if_exists="append")

def create_connection(db_file):
    """ create a database connection to a SQLite database """
    global conn
    conn = None
    try:
        conn = sqlite3.connect(db_file)
    except Error as e:
        print(e)

async def main(timeout=5.0):
    """
    Main wrapper to download data. This function will run continuously or until max_scans is hit.
    """
    global num_downloaded
    global tmp_byte_array
    devices = await BleakScanner.discover(timeout)
    WOWLS = checkForWowl(devices)
    for WOWL in WOWLS:
        """
        Under development. The functions of importance here is downloadData() and parseWOWLData().
        Be wary of the optional reset parameter in downloadData - True will reset WOWL directly
        after connecting and code will error out. This is a development functionality in case
        the WOWL gets stuck transmitting data.
        """
        # if WOWL=="CD:6C:3F:90:18:CE":
        if True:
            try:
                tmp_byte_array=[]
                await downloadData(WOWL,num_requested=1,delete_records=False,update_firmware=False,reset=False) #optional reset parameter here
            except:
                print("Error in downloadData")

if __name__ == "__main__":
    """This will continuously run function main() with the given timout. Can add sleep command to 
    reduce scanning intervals. The variable 'max_scans' will stop the continuous run. This was added 
    for development and can be removed in deployment.
    """
    max_scans,cnt=1,0
    create_connection("WOWLRecords.db")
    while True:
        try:
            start_time=time.time()
            asyncio.run(main())
            print("Scan #",cnt+1,"Elapsed Time:",time.time()-start_time)
        except:
            print("ctr c pressed (or far worse error)")
        time.sleep(0)
        cnt+=1
        if cnt>=max_scans:
            print("Scanned "+str(max_scans)+" Times, Terminating..")
            break
    if conn:
        conn.close()

"""
Other Notes:
-Can run this script fairly easily by editing /.bashrc file 
```DEPENDING ON OS IT MAY NOT BE THE BASHRC FILE IT COULD BE PROFILE OR SOMETHING STRANGE```
    -add sudo python [path_to_this_file]
-this should run everytime computer is turned on, and a new terminal is opened
-can add some sleeping to the file so it gets time to load everything properly before running?
"""