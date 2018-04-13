#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


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

bridge = CvBridge()

cv_image = None
media = []
centro = []

media2 = []
centro2 = []
atraso = 1.5


check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados
######======> Lendo primeira cor
def indentifica_cor1(frame):
	'''
	Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
	'''

	# No OpenCV, o canal H vai de 0 até 179, logo cores similares ao
	# vermelho puro (H=0) estão entre H=-8 e H=8.
	# Precisamos dividir o inRange em duas partes para fazer a detecção
	# do vermelho:
	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	cor_menor = np.array([0, 50, 50])
	cor_maior = np.array([8, 248, 255])
	segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

	cor_menor = np.array([170, 50, 50])
	cor_maior = np.array([180, 248, 255])
	segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)


	# A operação MORPH_CLOSE fecha todos os buracos na máscara menores
	# que um quadrado 7x7. É muito útil para juntar vários
	# pequenos contornos muito próximos em um só.
	segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

	# Encontramos os contornos na máscara e selecionamos o de maior área
	#contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	maior_contorno = None
	maior_contorno_area = 0

	for cnt in contornos:
	    area = cv2.contourArea(cnt)
	    if area > maior_contorno_area:
	        maior_contorno = cnt
	        maior_contorno_area = area

	# Encontramos o centro do contorno fazendo a média de todos seus pontos.
	if not maior_contorno is None :
	    cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
	    maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
	    media = maior_contorno.mean(axis=0)
	    media = media.astype(np.int32)
	    cv2.circle(frame, tuple(media), 5, [0, 255, 0])
	else:
	    media = (0, 0)

	cv2.imshow('video', frame)
	cv2.imshow('seg', segmentado_cor)
	cv2.waitKey(1)

	centro = (frame.shape[0]//2, frame.shape[1]//2)

	return media, centro



#####====> Lendo segunda cor :

def indentifica_cor2(frame):
	'''
	Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
	'''

	# No OpenCV, o canal H vai de 0 até 179, logo cores similares ao
	# vermelho puro (H=0) estão entre H=-8 e H=8.
	# Precisamos dividir o inRange em duas partes para fazer a detecção
	# do vermelho:
	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	cor_menor = np.array([120, 50, 50])
	cor_maior = np.array([128, 248, 255])
	segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

	cor_menor = np.array([112, 50, 50])
	cor_maior = np.array([120, 248, 255])
	segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)


	# A operação MORPH_CLOSE fecha todos os buracos na máscara menores
	# que um quadrado 7x7. É muito útil para juntar vários
	# pequenos contornos muito próximos em um só.
	segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

	# Encontramos os contornos na máscara e selecionamos o de maior área
	#contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	maior_contorno = None
	maior_contorno_area = 0

	for cnt in contornos:
	    area = cv2.contourArea(cnt)
	    if area > maior_contorno_area:
	        maior_contorno = cnt
	        maior_contorno_area = area

	# Encontramos o centro do contorno fazendo a média de todos seus pontos.
	if not maior_contorno is None :
	    cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
	    maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
	    media2 = maior_contorno.mean(axis=0)
	    media2 = media2.astype(np.int32)
	    cv2.circle(frame, tuple(media2), 5, [0, 255, 0])
	else:
	    media2 = (0, 0)

	cv2.imshow('video', frame)
	cv2.imshow('seg', segmentado_cor)
	cv2.waitKey(1)

	centro2 = (frame.shape[0]//2, frame.shape[1]//2)

	return media2, centro2


def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	#global media
	#global centro
	global media2
	global centro2
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
		media, centro = indentifica_cor1(closing)
 		media2, centro2 = indentifica_cor2(closing)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)



if __name__=="__main__":

	rospy.init_node("cor")
	# Para usar a Raspberry Pi
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)

	# Para usar a webcam
	recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if len(media) != 0 and len(centro) != 0:
				dif_x = media[0]-centro[0]
				dif_y = media[1]-centro[1]
				if math.fabs(dif_x)<30: # Se a media estiver muito proxima do centro anda para frente
					vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0.5))
					
				else:
					if dif_x > 0: # Vira a direita
						vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
					else: # Vira a esquerda
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))

			if len (media2) !=0 and len (centro2) != 0:
				dif_x = media2[0] - centro2 [0]
				dif_y = media2[1] - centro2 [1] 
				if math.fabs(dif_x)<30: # Se a media estiver muito proxima do centro anda para frente
					vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,1))


				rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
