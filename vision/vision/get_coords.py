import numpy as np
import cv2

# load a color image 
img = cv2.imread('board.png')
# get the image width and height
img_width = img.shape[0]
img_height = img.shape[1]

# turn into grayscale
img_g =  cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# turn into thresholded binary
_, thresh1 = cv2.threshold(img_g, 127, 255, cv2.THRESH_BINARY)

# find and draw contours. retr_external retrieves only the extreme outer contours
contours, _ = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(img, contours, -1, (0, 255, 0), 15)

tilecount = 0
for cnt in contours:
        tilecount += 1
        # use boundingrect to get coordinates of tile
        x, y, w, h = cv2.boundingRect(cnt)
        # create new image from binary, for further analysis. trim off the edge that has a line
        tile = thresh1[x + 40 : x + w - 80, y + 40 : y + h - 80]
        # create new image from main image, so we can draw the contours easily
        imgtile = img[x + 40 : x + w - 80, y + 40 : y + h - 80]

        #determine the array indexes of the tile
        tilex = round((x / img_width) * 3)
        tiley = round((y / img_height) * 3)     

        # put a number in the tile
        print(f'tilecount: {tilecount} = ({x + 80}, {y + 200})')
        cv2.putText(img, str(tilecount), (x + 80, y + 200), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 0, 0), 10)

# display image and release resources when key is pressed
cv2.imshow('image2', img)
cv2.waitKey(0)
cv2.destroyAllWindows()