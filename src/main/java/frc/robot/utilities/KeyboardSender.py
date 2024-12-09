import ntcore
import time
import keyboard

instance = ntcore.NetworkTableInstance.getDefault()
table = instance.getTable("Keyboard")
instance.startClient4("Keyboard")
instance.startDSClient()
keysToSend = "1234567890"

def connectionLoop():
    while not instance.isConnected():
        print("Not connected :(")
        time.sleep(1)
    for key in keysToSend:
        table.putBoolean(key, False)
    print("Connected :)")
    while instance.isConnected():
        event = keyboard.read_event()
        if event.name and event.name in keysToSend:
            table.putBoolean(event.name, event.event_type == keyboard.KEY_DOWN)
    connectionLoop()
    
connectionLoop()