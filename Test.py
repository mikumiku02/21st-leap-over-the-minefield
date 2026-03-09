import cv2
#import numpy as np


img=cv2.imread('105e18a37e65929c8597b10f0584e436.jpg', cv2.IMREAD_GRAYSCALE) # cv2.IMREAD_GRAYSCALE
print("图像数据", img.size)
cv2.namedWindow("test")
# cv2.resizeWindow("test", 108, 167)
# cv2.imshow("明日香", img)

#区域截取
# mrx=img[0:50, 0:200]
# cv2.imshow("明日香", mrx)

#颜色通道
# cur_img = img.copy()
# cur_img[:, :, 0] = 0
# cur_img[:, :, 1] = 0
# cv2.imshow("R", cur_img)

#图片拓展
##img = img + 10
# top_size, bottom_size, left_size, right_size = (50, 50, 50, 50)
# replicate = cv2.copyMakeBorder(img, top_size, bottom_size, left_size, right_size, borderType=cv2.BORDER_REPLICATE)#复制法
# replicate = cv2.copyMakeBorder(img, top_size, bottom_size, left_size, right_size, cv2.BORDER_REFLECT)# 反射法1
# replicate = cv2.copyMakeBorder(img, top_size, bottom_size, left_size, right_size, cv2.BORDER_REFLECT_101)# 反射法2
# replicate = cv2.copyMakeBorder(img, top_size, bottom_size, left_size, right_size, cv2.BORDER_WRAP)#外包装法
# replicate = cv2.copyMakeBorder(img, top_size, bottom_size, left_size, right_size, cv2.BORDER_CONSTANT, value=0)#常量法
# cv2.imshow("复制", replicate)

#图片融合
# 改shape值
# img=cv2.resize(img, (500, 414))

#图像阈值
ret, dst = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
# src:输入图，只能输入单通道图像，通常来说是灰度图
# thresh:阈值
# maxval:255
# type:
# 类型

#均值滤波
# blur = cv2.blur(img, (3, 3))

#腐蚀
# kernel = np.ones((5, 5), np.uint8)
# erosion = cv2.erode(dst, kernel, iterations = 1)
# 膨胀      cv2.dilate
#开：先腐蚀，再膨胀
# kernel = np.ones((5, 5), np.uint8)
# opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
#闭：                              cv2.MORPH_CLOSE

#梯度运算（膨胀-收缩）
# kernel = np.ones((7, 7), np.uint8)
# dilate = cv2.dilate(img, kernel, iterations=5)
# erosion = cv2.erode(img, kernel, iterations=5)

# res = np.hstack((dilate, erosion))
# gradient = cv2.morphologyEx(img, cv2.MORPH_GRADIENT,kernel)

#礼帽=原始输入-开运算结果               cv2.MORPH_TOPHAT
#黑帽=闭运算结果-原始输入               cv2.MORPH_BLACKHAT

#Sobel算子
# suan = cv2.Sobel(dst, cv2.CV_64F, 1, 0, ksize=3)
# suan = cv2.convertScaleAbs(suan)

# zi = cv2.Sobel(dst, cv2.CV_64F, 0, 1,ksize=3)
# zi = cv2.convertScaleAbs(zi)
# sz = cv2.addWeighted(suan, 0.5, zi, 0.5,0)

#Scharr算子,laplacian算子
# scharrx = cv2.Scharr(dst, cv2.CV_64F, 1, 0)
# scharry = cv2.Scharr(dst, cv2.CV_64F, 0, 1)
# scharrx = cv2.convertScaleAbs(scharrx)
# scharry = cv2.convertScaleAbs(scharry)
# scharrxy = cv2.addWeighted(scharrx, 0.5, scharry, 0.5, 0)

# laplacian = cv2.Laplacian(dst, cv2.CV_64F)
# laplacian = cv2.convertScaleAbs(laplacian)
# res = np.hstack((sz, scharrxy, laplacian))

#Canny边缘检测
#1）使用高斯滤波器，以平滑图像，滤除噪声
#2）计算图像中每一个像素点的梯度强度和方向
#3）应用非极大值（NOn-Maximun Suppression）抑制，以消除边缘检测带来的杂散响应
#4）应用双阈值（Double-Threshold）检测来确定真实的和潜在的边缘
#5）通过抑制孤立的边缘最终完成边缘检测
# v1 = cv2.Canny(dst, 80, 150)

#图像轮廓
# mode：轮廓检索模式
#.RETR_EXTERNAL：只检索最外面的轮廓；
#.RETR_LIST：检索所有的轮廓，并将其保存到一条链表当中；
#.RETRCCOMP：检索所有的轮廓，并将他们组织为两层：顶层是各部分的外部边界，第二层是空洞的边界；
#.RETR_TREE：检索所有的轮廓，并重构嵌套轮廓的整个层次；

#method：轮廓逼近方法
#.CHAIN_APPROX_NONE：以Freeman链码的方式输出轮廓，所有其他方法输出多边形(顶点的序列).
#.CHAIN_APPROX_SIMPLE：压缩水平的，垂直的和斜的部分，也就是，函数只保留他们的终点部分。\

# binary, contours, hierarchy = cv2.findContours(dst, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
# draw_img = dst.copy()
# res = cv2.drawContours(draw_img, contours, -1, (0, 0, 255), 2)

#模板匹配
#TM_SQDIFF: 计算平方不同，计算出来的值越小，越相关
#TM_CCORR: 计算相关性，计算出来的值越大，越相关
#TM_CCOEFF: 计算相关系数，计算出来的值越大，越相关
#TM_SQDIFF_NORMED: 计算归一平方不同，计算出来的值越接近0，越相关
#TM_CCORR_NORMED: 计算归一化相关性，计算出来的值越接近1，越相关
#TM_CCOEFF_NORMED: 计算归一化相关系数，计算出来的越接近1，越相关

#图像金字塔与轮廓检测
#up = cv2.pyrUp(img)
#down = cv2.pyrDown(img)
#Li = Gi-PyrUp(PyrDown(Gi))
#Gi是输入
down = cv2.pyrDown(img)
down_up = cv2.pyrUp(down, dstsize=(img.shape[1], img.shape[0]))
li = img-down_up

cv2.imshow('test', li)
print(img.shape)
cv2.waitKey(6000)

