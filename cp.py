import cv2

cv2.namedWindow('video', cv2.WINDOW_AUTOSIZE)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    img = cv2.cvtColor(frame, cv2.IMREAD_GRAYSCALE)  # cv2.IMREAD_GRAYSCALE,cv2.COLOR_BGR2GRAY

    ret1, dst = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    #suan = cv2.Sobel(dst, cv2.CV_64F, 0, 1, ksize
    #suan = cv2.convertScaleAbs(suan)

    #zi = cv2.Sobel(dst, cv2.CV_64F, 0, 1, ksize=3)
    #zi = cv2.convertScaleAbs(zi)

    #sz = cv2.addWeighted(suan, 0.5, zi, 0.5, 0)
    v1 = cv2.Canny(img, 80, 150)

    #binary, contours, hierarchy = cv2.findContours(dst, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    #draw_img = dst.copy()
    # 会在原图上画框并保存，一定要有draw_img = img.copy()
    #res = cv2.drawContours(draw_img, contours, -1, (0, 0, 255), 2)

    #down = cv2.pyrDown(dst)
    #down_up = cv2.pyrUp(down, dstsize=(img.shape[1], img.shape[0]))
    #li = dst-down_up
    cv2.imshow('video', v1)

    key = cv2.waitKey(2)
    if(key & 0xFF  ==  ord('q')):
        break

cap.release()
cv2.destroyAllWindows()
