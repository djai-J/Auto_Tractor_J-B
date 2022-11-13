import pygame
import button

import asyncio
from bleak import BleakClient


address = "34:08:E1:1A:03:62"
MODEL_NBR_UUID = "0522506A-7F0E-90AF-18DA-3037D2FEBFAE"

client = BleakClient(address) 
async def main(address):
	
	try:
		await client.connect()
		print('Connected again!')
	except Exception as e:
		print(e)

pygame.font.init()

#create display window
SCREEN_HEIGHT = 500
SCREEN_WIDTH = 800

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption('Button Demo')

#Authorship comments
font2 = pygame.font.SysFont('chalkduster.ttf', 30)
text1 = font2.render("Authorship: Jai Deshmukh and Brendan Love", True, (10, 10, 10))
textRect1 = text1.get_rect()
textRect1.center = ( 250, 250 )
#textpos = text.get_rect(centerx=background.get_width() / 2, y=10)


#load button images
start_img = pygame.image.load('start_btn.png').convert_alpha()
exit_img = pygame.image.load('exit_btn.png').convert_alpha()
stop_img = pygame.image.load('stop_btn.png') 


#create button instances
start_button = button.Button(50, 20, start_img, 0.5)
stop_button = button.Button(250, 20, stop_img, 0.5)
exit_button = button.Button(450, 20, exit_img, 0.5)

#game loop
run = True
asyncio.run(main(address)) 
while run:


	#asyncio.run(main(address))  



	screen.fill((202, 228, 241))
	screen.blit(text1, textRect1) 

	if start_button.draw(screen):
		print('START')
	if stop_button.draw(screen):
		print('STOP')
	if exit_button.draw(screen):
		print('EXIT')
		client.disconnect()
		print('Disconnected')
		run = False



	#event handler
	for event in pygame.event.get():
		#quit game
		if event.type == pygame.QUIT:
			run = False

	pygame.display.update()



pygame.quit()