import cv2

# file ini hanya untuk mendapatkan koordinat tiap kotak di board

img = cv2.imread('board.png')
# panjang dan lebar gambar board
img_width = img.shape[0]
img_height = img.shape[1]

img_g =  cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
_, thresh1 = cv2.threshold(img_g, 127, 255, cv2.THRESH_BINARY)

# detect & gambar contour
contours, _ = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(img, contours, -1, (0, 255, 0), 15)

tilecount = 0
for cnt in contours:
        tilecount += 1
        
        # koordinat tiap tile
        x, y, w, h = cv2.boundingRect(cnt)  

        # percobaan taro teks di koordinat
        print(f'tilecount: {tilecount} = ({x + 80}, {y + 200})') # tambahan angka untuk menyempurnakan penempatan
        cv2.putText(img, str(tilecount), (x + 80, y + 200), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 0, 0), 10)

# display image
cv2.imshow('image2', img)
cv2.waitKey(0)
cv2.destroyAllWindows()