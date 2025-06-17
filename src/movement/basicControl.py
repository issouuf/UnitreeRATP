import struct
import paho.mqtt.client as mqtt
import time

client = mqtt.Client()
client.connect("192.168.12.1", 1883, 60)
client.publish("controller/action", "walk")

# 1er param : Mouvement Gauche ou Droite
# -1 = Gauche à fond (~1m)
#  1 = Droite à fond (~1m)

# 2eme param : Rotation Gauche ou Droite
# -1 = Rotation vers la gauche (~180°)
#  1 = Rotation vers la droite (~180°)

# 3eme param : ? Pitche/Yaw (Ne fais rien)

# 4eme param : Mouvement Arrière ou Avant
# -1 = Arrière à fond (~1m)
#  1 = Avant à fond (~1m)

while True:
   payload = struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0)
   client.publish("controller/stick", payload)
   time.sleep(4)