from perceptbot_camera.msg import ObjDetection

PATH = "/home/pi/OpenVino/Models/"


def obj_detection_msg(x, y, height, width, class_label, confidence):
    return ObjDetection(xmin=(x - width / 2),
                        ymin=(y - height / 2),
                        xmax=(x + width / 2),
                        ymax=(y + height / 2),
                        class_label=class_label,
                        confidence=confidence)


def intersection_over_union(box_1, box_2):
    width_of_overlap_area = min(box_1.xmax, box_2.xmax) - max(box_1.xmin, box_2.xmin)
    height_of_overlap_area = min(box_1.ymax, box_2.ymax) - max(box_1.ymin, box_2.ymin)
    if width_of_overlap_area < 0.0 or height_of_overlap_area < 0.0:
        area_of_overlap = 0.0
    else:
        area_of_overlap = width_of_overlap_area * height_of_overlap_area
    box_1_area = (box_1.ymax - box_1.ymin) * (box_1.xmax - box_1.xmin)
    box_2_area = (box_2.ymax - box_2.ymin) * (box_2.xmax - box_2.xmin)
    area_of_union = box_1_area + box_2_area - area_of_overlap
    return area_of_overlap / area_of_union


def filter_overlapping_boxes(objects):
    obj_len = len(objects)
    for i in range(obj_len):
        if objects[i].confidence == 0.0:
            continue
        for j in range(i + 1, obj_len):
            if objects[i].class_label == objects[j].class_label and \
                    (intersection_over_union(objects[i], objects[j]) >= 0.4):
                objects[j].confidence = 0
    return [obj for obj in objects if obj.confidence > 0]
