#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cor_vermelho
import cor_azul
import scanner
from sensor_msgs.msg import LaserScan
global morto
morto=False

bridge = CvBridge()

cv_image = None
mediaR = []
centroR = []
mediaB = []
centroB = []

atraso = 1.5



check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global mediaR
	global centroR
	global mediaB
	global centroB
	kernel = np.ones((5,5),np.uint8)

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		opening = cv2.morphologyEx(cv_image, cv2.MORPH_OPEN, kernel)
		closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
		mediaR, centroR = cor_vermelho.indentifica_cor1(closing)
		mediaB, centroB = cor_azul.indentifica_cor2(closing)
 		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

def scaneou(dado):
#	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
#	print("Leituras:")
	lista_dados=np.array(dado.ranges).round(decimals=2)
	morto=False
	iteracoes = 0
	atencao = 0.3
	maxi = max(lista_dados)
	for i in range(len(lista_dados)):
		if lista_dados[i] == 0:
			lista_dados[i] = 100
		else:
			iteracoes += 1

	mini = min(lista_dados)

	for i in range(len(lista_dados)):
		if mini < atencao:
			print("AZEDOU")
			morto=True
			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
		else:
			print("OK")
			morto = False


if __name__=="__main__":

	rospy.init_node("cor")
	# Para usar a Raspberry Pi
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	# Para usar a webcam
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			if morto:
				velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(velocidade)
				rospy.sleep(2)
			else:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				if len(mediaR) != 0 and len(centroR) != 0:
					dif_x = mediaR[0]-centroR[0]
					dif_y = mediaR[1]-centroR[1]
					if math.fabs(dif_x)<30:

	 # Se a mediaR estiver muito proxima do centro anda para frente
						vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,0))
					else:
						if dif_x > 0: # Vira a direita
							vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
						else: # Vira a esquerda
							vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
				velocidade_saida.publish(vel)
				rospy.sleep(0.01)

				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				if len(mediaB) != 0 and len(centroB) != 0:
					dif_x = mediaB[0]-centroB[0]
					dif_y = mediaB[1]-centroB[1]
					if math.fabs(dif_x)<30: # Se a mediaB estiver muito proxima do centro anda para frente
						vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
					else:
						if dif_x > 0: # Vira a direita
							vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
						else: # Vira a esquerda
							vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
				velocidade_saida.publish(vel)
				rospy.sleep(0.01)




	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
