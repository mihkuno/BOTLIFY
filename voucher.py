#!/home/botlify/BOTLIFY/venv/bin/python

import json
import os
import sys
import display
import time

METHOD = sys.argv[1]
MAC = sys.argv[2]

if METHOD == "auth_client":
    USERNAME = sys.argv[3]
    PASSWORD = sys.argv[4]
    
    voucher = ''
    minutes = 0
    output_directory = '/home/botlify/BOTLIFY/output'
    data_file_path = os.path.join(output_directory, 'data.json')
    
    # Read then increment time
    with open(data_file_path, 'r') as file:
        data = json.load(file)
        voucher = data['voucher']
        minutes = data['minutes']*60 # minute to seconds
    
    if USERNAME == "default" and PASSWORD == voucher:
        # Allow client to access the Internet for one hour (3600 seconds)
        # Further values are upload and download limits in bytes. 0 for no limit.
        print(f"{minutes} 0 0")
        
        # after user connected, reset the voucher data
        with open(data_file_path, 'w+') as file:
            json.dump({"box": [], "label": "", "score": 0, "area": 0, "minutes": 0, "voucher": ""}, file)
         
        display.thank_you()
        time.sleep(3)           
        display.insert_bottle_a()

        print('successful')
        sys.exit(0)
    else:
        print('wrong password')
        # Deny client to access the Internet.
        sys.exit(1)
elif METHOD in ["client_auth", "client_deauth", "idle_deauth", "timeout_deauth", "ndsctl_auth", "ndsctl_deauth", "shutdown_deauth"]:
    INGOING_BYTES = sys.argv[3]
    OUTGOING_BYTES = sys.argv[4]
    SESSION_START = sys.argv[5]
    SESSION_END = sys.argv[6]
    # client_auth: Client authenticated via this script.
    # client_deauth: Client deauthenticated by the client via splash page.
    # idle_deauth: Client was deauthenticated because of inactivity.
    # timeout_deauth: Client was deauthenticated because the session timed out.
    # ndsctl_auth: Client was authenticated by the ndsctl tool.
    # ndsctl_deauth: Client was deauthenticated by the ndsctl tool.
    # shutdown_deauth: Client was deauthenticated by Nodogsplash terminating.
else:
    print("Invalid method")
