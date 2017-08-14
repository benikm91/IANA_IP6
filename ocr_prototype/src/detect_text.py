# apt-get install tesseract-ocr-deu
import Image
from tesserocr import PyTessBaseAPI

import cv2
from extract_roi import text_detection
from extract_roi3 import number_detection, cutout_rectangles

output = 'temp/output.png'

debug = False

vars = {
    'user_words_suffix': 'user-words',
    #'tessedit_char_whitelist': 'MUSTAFA'
}


def number_recognition(img):
    cv2.imshow('lala', img)
    cv2.waitKey(0)

    _, img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 0.0, 255.0, cv2.THRESH_OTSU + cv2.THRESH_BINARY)

    cv2.imshow('lala', img)
    cv2.waitKey(0)

    morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, morph_kernel)

    img = cv2.dilate(img, (20, 20))

    cv2.imshow('lala', img)
    cv2.waitKey(0)

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    with PyTessBaseAPI() as api:
        api.InitFull(lang='deu', variables= {
            'user_words_suffix': 'user-words',
        })
        api.SetImage(Image.fromarray(img))
        result = api.GetUTF8Text()
        return result


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


def debug_image(img, rects):
    for rect in rects:
        cv2.rectangle(img, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
    cv2.imshow("img", img)
    cv2.waitKey(0)


# img_path = 'res/dlink_zoom_1_5m_2017-06-27.jpg'
# img_path = 'res/dlink_zoom_medium_color_2m.jpg'
# img_path = 'res/dlink_zoom_medium_high_color_2m.jpg'
# img_path = 'res/dlink_zoom_medium_medium_high_color_2m.jpg'
img_path = 'res/dlink_zoom_color_2m.jpg'
# img_path = "res/dlink_zoom_medium_high_color_2m.jpg"
img = cv2.imread(img_path)

cv2.imshow("img", img)
cv2.waitKey(0)

text_highlight_img = cv2.copyMakeBorder(img,0,0,0,0,cv2.BORDER_REPLICATE)

rects = text_detection(img)
rects.sort(key=lambda r: r[1])
debug_image(img, rects)
text_images = cutout_rectangles(img, rects)
print "Number of texts found: {0}".format(len(text_images))
texts = list()
for text_image in text_images:
    texts.append(text_recognition(text_image))
texts = connect_texts(texts)
for text in texts:
    print text
cv2.waitKey(0)

rects = number_detection(img)
rects.sort(key=lambda r: r[1])
debug_image(img, rects)
text_images = cutout_rectangles(img, rects)
print "Number of texts found: {0}".format(len(text_images))
texts = list()
for text_image in text_images:
    texts.append(number_recognition(text_image))
for text in texts:
    print text
cv2.waitKey(0)


cv2.destroyAllWindows()
