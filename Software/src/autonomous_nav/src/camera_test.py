import cv2

cam = cv2.VideoCapture(2)

while True:
	ret, image = cam.read()
	cv2.imshow('Imagetest',image)
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
		
    
cv2.imwrite('/home/pi/testimage.jpg', image)
cam.release()
cv2.destroyAllWindows()