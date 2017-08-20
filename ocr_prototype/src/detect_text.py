# apt-get install tesseract-ocr-deu
import Image
from tesserocr import PyTessBaseAPI

import cv2
import numpy as np
from extract_roi import text_detection
from extract_roi3 import number_detection


debug_counter = 0
def debug_image(title, img):
    global debug_counter
    cv2.imshow(str(debug_counter)+title+".png", img)
    cv2.waitKey(0  )
    debug_counter += 1


def cutout_rectangles(img, rects):
    result = list()
    delta = 5
    height, width = img.shape[:2]
    for rect in rects:
        x = max(rect[1] - delta, 0)
        xx = min(rect[1] + rect[3] + delta, height - 1)
        y = max(rect[0] - delta, 0)
        yy = min(rect[0] + rect[2] + delta, width - 1)
        print x, xx ,y, yy
        result.append(img[x:xx, y:yy])
    return result


# http://www.pyimagesearch.com/2017/02/20/text-skew-correction-opencv-python/
def deskew(img):
    def get_deskew_angle():
        coords = np.column_stack(np.where(img == 0))
        angle = cv2.minAreaRect(coords)[-1]

        if angle < -45:
            angle = -(90 + angle)
        else:
            angle = -angle
        return angle

    (h, w) = img.shape[:2]
    M = cv2.getRotationMatrix2D((w // 2, h // 2), get_deskew_angle(), 1.0)
    img = cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
    return img

output = 'temp/output.png'

debug = False

vars = {
    'user_words_suffix': 'user-words',
    #'tessedit_char_whitelist': 'MUSTAFA'
}


def number_recognition(img):
    with PyTessBaseAPI() as api:
        api.InitFull(lang='deu', variables= {
            'user_words_suffix': 'user-words',
            #'psm':   str(7),
        })
        api.SetImage(Image.fromarray(img))
        result = api.GetUTF8Text()
        return  result


def text_recognition(img):
    with PyTessBaseAPI() as api:
        api.InitFull(lang='deu', variables= {
            'user_words_suffix': 'user-words',
        })
        api.SetImage(Image.fromarray(img))
        result = api.GetUTF8Text()
        return result


def connect_texts(texts):

    connections = [-1 for _ in range(0, len(texts))]

    assert len(connections) == len(texts) and len(texts) == len(rects)

    # find connections
    for i in range(0, len(rects)):
        for j in range(0, len(rects)):
            if i != j:
                rect1 = rects[i]
                rect2 = rects[j]
                if pow(rect1[0] - rect2[0], 2) + pow(rect1[1] + rect1[3] - rect2[1], 2) < 200:
                    connections[i] = j

    # apply connections
    for i in range(0, len(connections)):
        if connections[i] != -1:
            if texts[i] != "" and texts[connections[i]] != "":
                texts[i] = texts[i] + " " + texts[connections[i]]
                texts[connections[i]] = ""

    import re

    texts = map(lambda text: re.sub('\s+', ' ', text.replace('\n', '  ')).strip(), texts)
    return filter(None, texts)


def debug_image_with_rects(name, img, rects):
    lala = img.copy()
    for rect in rects:
        cv2.rectangle(lala, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
    debug_image(name, lala)


# img_path = 'res/dlink_zoom_1_5m_2017-06-27.jpg'
# img_path = 'res/dlink_zoom_medium_color_2m.jpg'
# img_path = 'res/dlink_zoom_medium_high_color_2m.jpg'
# img_path = 'res/dlink_zoom_medium_medium_high_color_2m.jpg'
img_path = " _set/01.jpg"
# img_path = "res/dlink_zoom_medium_high_color_2m.jpg"
img = cv2.imread(img_path)

debug_image("original", img)

text_highlight_img = cv2.copyMakeBorder(img,0,0,0,0,cv2.BORDER_REPLICATE)

rects = number_detection(img)
rects.sort(key=lambda r: r[1])
debug_image_with_rects("detected_texts", img, rects )
# debug_image(img, rects)
text_images = cutout_rectangles(img, rects)
print "Number of texts found: {0}".format(len(text_images))
texts = list()
for text_image in text_images:
    #debug_image("number_image_01", text_image)
    _, text_image = cv2.threshold(cv2.cvtColor(text_image, cv2.COLOR_BGR2GRAY), 0.0, 255.0, cv2.THRESH_OTSU + cv2.THRESH_BINARY)
    #debug_image("number_image_02", text_image)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (12, 12))
    text_image = cv2.morphologyEx(text_image, cv2.MORPH_CLOSE, kernel)
    #debug_image("number_image_03", text_image)
    text_image = deskew(text_image)
    #debug_image("number_image_04", text_image)
    texts.append(number_recognition(text_image))
for text in texts:
    print text
cv2.waitKey(0)

rects = text_detection(img)
rects.sort(key=lambda r: r[1])
debug_image_with_rects("detected_texts", img, rects )
text_images = cutout_rectangles(img, rects)
texts = list()
print "Number of texts found: {0}".format(len(text_images))
for text_image in text_images:
    debug_image("text_image_01", text_image)
    _, text_image = cv2.threshold(cv2.cvtColor(text_image, cv2.COLOR_BGR2GRAY), 0.0, 255.0, cv2.THRESH_OTSU + cv2.THRESH_BINARY)
    debug_image("text_image_02", text_image)
    text_image = deskew(text_image)
    debug_image("text_image_03", text_image)
    texts.append(text_recognition(text_image))
texts = connect_texts(texts)
for text in texts:
    print text
cv2.waitKey(0)

cv2.destroyAllWindows()
