# apt-get install tesseract-ocr-deu

from tesserocr import PyTessBaseAPI

import cv2
import tesserocr
from extract_roi import text_detection, cutout_rectangles
from pyocr import tesseract

output = 'temp/output.png'

debug = False

vars = {
    'user_words_suffix': 'user-words',
    #'tessedit_char_whitelist': 'MUSTAFA'
}

def text_recognition(img, title, show_img = False):
    global vars
    cv2.imwrite(output, img)
    with PyTessBaseAPI() as api:
        api.InitFull(lang='deu', variables=vars)
        api.SetImageFile(output)
        result = api.GetUTF8Text()
        if show_img:
            cv2.imshow(title, img)
            cv2.waitKey(0)
        return result.strip()

# img_path = 'res/dlink_zoom_1_5m_2017-06-27.jpg'
# img_path = 'res/dlink_zoom_medium_color_2m.jpg'
img_path = 'res/dlink_zoom_medium_high_color_2m.jpg'
# img_path = 'res/dlink_zoom_medium_medium_high_color_2m.jpg'
# img_path = 'res/dlink_zoom_color_2m.jpg'
img = cv2.imread(img_path)

cv2.imshow("img", img)
cv2.waitKey(0)

text_highlight_img = cv2.copyMakeBorder(img,0,0,0,0,cv2.BORDER_REPLICATE)

rects = text_detection(img)

rects.sort(key=lambda r: r[1])

for rect in rects:
    cv2.rectangle(text_highlight_img, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
cv2.imshow("img", text_highlight_img)
cv2.waitKey(0)

text_images = cutout_rectangles(img, rects)

print "Number of texts found: {0}".format(len(text_images))

texts = list()
for text_image in text_images:
    texts.append(text_recognition(text_image, 'text_recognition', debug))

connections = [-1 for _ in range(0, len(texts))]

assert len(connections) == len(texts) and len(texts) == len(rects)

# find connections
for i in range(0, len(rects)):
    for j in range(0, len(rects)):
        if i != j:
            rect1 = rects[i]
            rect2 = rects[j]
            if pow(rect1[0] - rect2[0], 2) + pow(rect1[1] + rect1[3] - rect2[1], 2) < 20:
                connections[i] = j

# apply connections
for i in range(0, len(connections)):
    if connections[i] != -1:
        if texts[i] != "" and texts[connections[i]] != "":
            texts[i] = texts[i] + " " + texts[connections[i]]
            texts[connections[i]] = ""

texts = filter(None, texts)

for text in texts:
    print text

cv2.waitKey(0)

cv2.destroyAllWindows()
