from numpy import ndarray
import cv2 as cv
import numpy as np
from lane_follow_blob.utils import cols, rows
from lane_follow_blob.cfg import BlobConfig


def compute_lines(image: ndarray,
                  config: BlobConfig,
                  debug_image=None) -> ndarray:
    lines_mat = np.zeros_like(image)

    # Images to draw lines on 
    if debug_image is None:
        draw_lines_on = (lines_mat,)
    else:
        draw_lines_on = (lines_mat, debug_image)

    # Crop out the upper portion of the image
    # This helps with avoiding detecting lines
    # found in objects along the horizon (i.e. trees, posts)
    x = 0
    y = int(config.lines_top * rows(image))
    w = cols(image)
    h = int(rows(image) - config.lines_top * rows(image))
    image_cropped = image[y:y+h, x:x+w]

    lines = cv.HoughLinesP(image_cropped,
                            rho=config.lines_rho,
                            theta=0.01745329251,
                            threshold=config.lines_thresh,
                            minLineLength=config.lines_min_len,
                            maxLineGap=config.lines_max_gap)
    if lines is not None:
        for l in lines:
            l = l[0] # (4,1) => (4,)
            diffx = l[0] - l[2]
            diffy = l[1] - l[3]

            slope = diffy / diffx

            if abs(slope) < config.lines_min_slope or abs(slope) > config.lines_max_slope:
                continue

            diffx *= 5
            diffy *= 5

            l[0] -= diffx
            l[1] -= diffy
            l[2] += diffx
            l[3] += diffy

            for img in draw_lines_on:
                cv.line(img,
                    (l[0], int(l[1] + config.lines_top * rows(image))),
                    (l[2], int(l[3] + config.lines_top * rows(image))),
                    255, 5)

    return lines_mat



def find_lanes(input_image: ndarray,
               config: BlobConfig,
               debug_image: ndarray=None) -> ndarray:
    """
    This algorithm uses light-on-dark contrast to find 
    lane lines. If lanes do not have this property, another
    lane-finding algorithm may be used instead
    """

    # Median blur
    image = cv.medianBlur(input_image, config.enhance_blur * 2 + 1)
    
    
    image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        
    #thresh
    ret, image = cv.threshold(image, # input image
                        170,
                        255,    # max value in image
                        cv.THRESH_BINARY) 
    
    image = cv.medianBlur(image, config.enhance_blur * 2 + 1)

    ## Find edges using Laplacian and Sobel
    # Canny could also be used here but the Laplacian/Sobel
    # approach typically yeilds improved expiremental
    # results for this case
    # image = cv.Laplacian(image, -1, config.lapla_ksize * 2 + 1)
    # image = cv.Sobel(image, -1, config.sobel_xorder,
    #             config.sobel_yorder,
    #             config.sobel_ksize * 2 + 1)
    image = cv.Canny(image, config.canny_lower_thresh,
            config.canny_upper_thresh,
            apertureSize=config.canny_aperture_size * 2 + 1)

    # Dilate images
    dilation_size = (2 * config.blob_dilation_size + 1, 2 * config.blob_dilation_size + 1)
    dilation_anchor = (config.blob_dilation_size, config.blob_dilation_size)
    dilate_element = cv.getStructuringElement(cv.MORPH_RECT, dilation_size, dilation_anchor)
    image = cv.dilate(image, dilate_element)

    if config.lines_enable:
        image = compute_lines(image, config, debug_image=debug_image)

    return image