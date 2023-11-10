import paho.mqtt.client as mqtt
import time
import json
from gtts import gTTS
import os
import multiprocessing
#import logging
import random
#import playsound
import pygame

#CONFIGURAR DE ACORDO COM O BROKER
#broker = '192.168.233.156'
broker = "192.168.1.53"
#broker = 'localhost'
port = 1883
#topic = "testtopic/#" # For receiveing data
#topic = "/node1/#"
topic = "node1/#"
#topic_pub = "testtopic/Weather" # For publishing
topic_pub = "node1/amb"
client_id = f'python-mqtt-{random.randint(0, 1000)}'
# Se usar username:pass é preciso retirar o comentário na função de criação do objeto (main body, fase 1)
#username = "mqtt_user"
username = "iotdev"
#password = "123abc!"
password = "Qwe123!"
language = 'pt'
#stop_client = False
#cont_mess = 0
brokers_pub = {"time":0,"BME":{"Temp":23.45, "Hum":53.11}, "UV":5} # Como exemplo e para verificar se o broker aceita publicar

# Filas para comunicação entre processos
queueAmb = multiprocessing.Queue()
queueSnd = multiprocessing.Queue()
queueDist = multiprocessing.Queue()

########### Callback functions do MQTT_client
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.connected_flag = True # define a flag to mark a good connection
        print("Connected Ok.")
        client.subscribe(topic)
    else:
        print("Failed to connect. Return code %d\n", rc)
        client.bad_connection_flag = True # define a flag to mark a connection error

def on_message(client, userdata, msg):
    print("data Received at topic ", msg.topic)
    m_decode=str(msg.payload.decode("utf-8", "ignore"))
    #print("Converting from Json to Object")
    m_in = json.loads(m_decode)
    #print(m_in)
    if "amb" in msg.topic:
        queueAmb.put(m_in)
    else:
        if "snd" in msg.topic:
            queueSnd.put(m_in)
        else:
            if "dist" in msg.topic:
                queueDist.put(m_in)
            else:
                print("ClienteMQTT: mens desconhecida, ignorada")
###########
    
def Proc_mess(qSnd, qDist, qAmb):
    pygame.mixer.init()     # initialize pygame object
    # Mensagem inicial de boas vindas (estática)
    pygame.mixer.music.load("welcome.mp3")
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy() == True:
        continue
    #pygame.mixer.music.unload()
    #playsound.playsound("welcome.mp3")
    actUV = 0
    actTemp = 0
    actHum = 0
    actSnd = 0
    prevDist = [0, 0, 0, 0, 0, 0]
    actDist = [0, 0, 0, 0, 0, 0]
    ChangPos = ["far", "far", "far", "far", "far", "far"]
    while True: # Executa continuamente até o processo main fazer o join()
        while (qSnd.empty() and qDist.empty() and qAmb.empty()):
            time.sleep(0.5)
        if not qSnd.empty():
            while not qSnd.empty(): # consome toda a fila até ao último valor
                m_in = qSnd.get()
            actSnd = m_in["DB"]
            #print(actSnd)
        if not qDist.empty():
            while not qDist.empty(): # consome toda a fila até ao último valor
                m_in = qDist.get()
            #print(len(m_in["SDist"]))
            for i in (0, len(m_in["SDist"])-1):
                prevDist[i] = actDist[i]
                actDist[i] = m_in["SDist"]["id"+str(i)]
                #print(actDist[i], prevDist[i])
                if (actDist[i] < prevDist[i]):
                    ChangPos[i] = "near"
                else:
                    ChangPos[i] = "far"
            #print(actDist, ChangPos)
        if not qAmb.empty():
            while not qAmb.empty(): # consome toda a fila até ao último valor
                m_in = qAmb.get()
            #print(m_in)
            actUV = m_in["UV"]
            actTemp = m_in["BME"]["Temp"]
            actHum = m_in["BME"]["Hum"]
            #print(actTemp, actHum, actUV)
        if "near" in ChangPos:
            ChangPos = ["far", "far", "far", "far", "far", "far"] # Reset list de alterações
            mytext = "Neste momento a temperatura é de "+ str(int(actTemp))+ " graus centígrados, a humidade está a "+ str(int(actHum))+ " por cento e a radiação UV está no nível " + str(actUV)
            myobj = gTTS(text=mytext, lang=language, slow=False)
            myobj.save("tempo.mp3")
            pygame.mixer.music.load("tempo.mp3")
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy() == True:
                continue
            #pygame.mixer.music.unload()
            #playsound.playsound("tempo.mp3")
            os.remove("tempo.mp3")

# main body
if __name__ == "__main__":
    # Setup of logging schema (DEBUG)
    #logging.basicConfig(format='%(asctime)s - %(levelname)s: %(message)s',
    #                        level=logging.DEBUG
    # Connect to broker
    # 1. Create object
    client = mqtt.Client(client_id)
    client.username_pw_set(username, password)
    # Inicializa flags criadas na classe para controlar o estado da ligação -> loop_start()
    mqtt.Client.bad_connection_flag = False
    mqtt.Client.connected_flag = False

    # 2. Setup callback functions
    client.on_connect = on_connect  # Set callback function for connect message
    client.on_message = on_message  # Set callback function for regular messages
    # 3. Connecting
    print("Connecting to broker", broker)
    client.connect(broker, port, keepalive=60)
    while not client.is_connected:
        print("In wait loop")
        time.sleep(1)
    print("In main loop")
    #Publica na primeira tentativa para verificar se está tudo bem
    data_out = json.dumps(brokers_pub) # encode oject to JSON
    print("sending data")
    client.publish(topic_pub, data_out)
    # Inicia o processo que vai processar as mensagens recebidas
    message_proc = multiprocessing.Process(target=Proc_mess, args=(queueSnd, queueDist, queueAmb,))
    message_proc.start()
    # Entra no ciclo infinito para executar o cliente
    client.loop_forever()
    # Alternatica ao ciclo infinito para poder terminar de forma controlada. ainda não consegui fazer isso.
    #while not stop_client:
    #    time.sleep(1)
    #client.loop_stop()
    #client.disconnect()
