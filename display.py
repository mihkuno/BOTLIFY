
import os
import time


def clear_tty1():
    with open('/dev/tty1', 'w') as tty1:
        tty1.write('\033c')  # ANSI escape code to reset the terminal


def print_to_tty1(message):
    with open('/dev/tty1', 'w') as tty1:
        tty1.write(message + '\n')



def initializing():
    clear_tty1()
    print_to_tty1("""
\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n
       ___       _ _   _       _ _     _            
      |_ _|_ __ (_) |_(_) __ _| (_)___(_)_ __   __ _ 
       | || '_ \| | __| |/ _` | | |_  / | '_ \ / _` |
       | || | | | | |_| | (_| | | |/ /| | | | | (_| |
      |___|_| |_|_|\__|_|\__,_|_|_/___|_|_| |_|\__, |
                                               |___/ 
\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n
    """)


def redirecting():
    clear_tty1()
    print_to_tty1("""
\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n
      ____          _ _               _   _             
     |  _ \ ___  __| (_)_ __ ___  ___| |_(_)_ __   __ _ 
     | |_) / _ \/ _` | | '__/ _ \/ __| __| | '_ \ / _` |
     |  _ <  __/ (_| | | | |  __/ (__| |_| | | | | (_| |
     |_| \_\___|\__,_|_|_|  \___|\___|\__|_|_| |_|\__, |
                                                  |___/ 
            
                          .  .  .
\n\n\n\n\n\n\n\n\n\n\n\n\n\n
    """)
    
    
def please_wait():
    clear_tty1()
    print_to_tty1("""
\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n                  
         _                                     _ _         
   _ __ | | ___  __ _ ___  ___  __      ____ _(_| |_       
  | '_ \| |/ _ \/ _` / __|/ _ \ \ \ /\ / / _` | | __|      
  | |_) | |  __| (_| \__ |  __/  \ V  V | (_| | | |_ _ _ _ 
  | .__/|_|\___|\__,_|___/\___|   \_/\_/ \__,_|_|\__(_(_(_)
  |_|                                                      
\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n                                                              
    """)
    
    
def thank_you():
    clear_tty1()
    print_to_tty1("""
\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n                  
      _   _                 _                        
     | |_| |__   __ _ _ __ | | __  _   _  ___  _   _ 
     | __| '_ \ / _` | '_ \| |/ / | | | |/ _ \| | | |
     | |_| | | | (_| | | | |   <  | |_| | (_) | |_| |
      \__|_| |_|\__,_|_| |_|_|\_\  \__, |\___/ \__,_|
                                   |___/             
\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n                                                              
    """)
    


def invalid():
    clear_tty1()
    print_to_tty1("""
\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n                  
             _                 _ _     _ 
            (_)_ ____   ____ _| (_) __| |
            | | '_ \ \ / / _` | | |/ _` |
            | | | | \ V | (_| | | | (_| |
            |_|_| |_|\_/ \__,_|_|_|\__,_|
                                       
\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n                                                              
    """)
    
      
def insert_bottle_b(voucher, minutes):
    h = minutes // 60
    m = minutes % 60
    
    clear_tty1()
    print_to_tty1(f"""
\n\n\n\n\n\n\n\n\n\n\n
         ██ ███    ██ ███████ ███████ ██████  ████████     
         ██ ████   ██ ██      ██      ██   ██    ██        
         ██ ██ ██  ██ ███████ █████   ██████     ██        
         ██ ██  ██ ██      ██ ██      ██   ██    ██        
         ██ ██   ████ ███████ ███████ ██   ██    ██        
 
      ██████   ██████  ████████ ████████ ██      ███████    
      ██   ██ ██    ██    ██       ██    ██      ██         
      ██████  ██    ██    ██       ██    ██      █████      
      ██   ██ ██    ██    ██       ██    ██      ██         
      ██████   ██████     ██       ██    ███████ ███████    

                                                     

       Open your wifi settings and connect to 'PLASTIFI-HUB' 

                Time:       {h} hour {m} minutes

                Voucher:    {voucher}

    """)


def insert_bottle_a():
    clear_tty1()
    print_to_tty1("""
\n\n\n\n\n\n\n\n\n\n\n
         ██ ███    ██ ███████ ███████ ██████  ████████     
         ██ ████   ██ ██      ██      ██   ██    ██        
         ██ ██ ██  ██ ███████ █████   ██████     ██        
         ██ ██  ██ ██      ██ ██      ██   ██    ██        
         ██ ██   ████ ███████ ███████ ██   ██    ██        
 
      ██████   ██████  ████████ ████████ ██      ███████    
      ██   ██ ██    ██    ██       ██    ██      ██         
      ██████  ██    ██    ██       ██    ██      █████      
      ██   ██ ██    ██    ██       ██    ██      ██         
      ██████   ██████     ██       ██    ███████ ███████    
\n\n\n\n\n\n\n\n\n\n\n
    """)
