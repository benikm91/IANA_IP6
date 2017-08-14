import cv2

def text_detection(rgb, debug = False):
    def show_debug_img(title, image):
        if debug:
            cv2.imshow(title, image)
            cv2.waitKey(0)

    show_debug_img('t', rgb)

    vis = rgb.copy()

    for i in range(0, 80):

        mser = cv2.MSER_create(_delta=i)
        regions = mser.detectRegions(rgb)

        hulls = [cv2.convexHull(p[0].reshape(-1, 1, 2)) for p in regions]

        cv2.fillPoly(vis, hulls, (0, 255, 0))
        cv2.imshow('img', vis)
        cv2.waitKey(0)

    show_debug_img('t', rgb)


if __name__ == "__main__":
    img_path = "../res/a.jpg  "
    # img_path = "../res/dlink_zoom_color_2m.jpg"
    large = cv2.imread(img_path, 0)
    rects = text_detection(large, True)
    #for rect in rects:
    #    cv2.rectangle(large, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
    cv2.destroyAllWindows()

