

class YbProtocol:
    def __init__(self):
        self.ID_COLOR = 1
        self.ID_BARCODE = 2
        self.ID_QRCODE = 3
        self.ID_APRILTAG = 4
        self.ID_DMCODE = 5
        self.ID_FACE_DETECT = 6
        self.ID_EYE_GAZE = 7
        self.ID_FACE_RECOGNITION = 8
        self.ID_PERSON_DETECT = 9
        self.ID_FALLDOWN_DETECT = 10
        self.ID_HAND_DETECT = 11
        self.ID_HAND_GESTURE = 12
        self.ID_OCR_REC = 13
        self.ID_OBJECT_DETECT = 14
        self.ID_NANO_TRACKER = 15
        self.ID_SELF_LEARNING = 16
        self.ID_LICENCE_REC = 17
        

    def package_coord(self, func, x, y, w, h, msg=None):
        pto_len = 0
        if msg is None:
            temp_buf = "$%02d,%02d,%03d,%03d,%03d,%03d#" % (pto_len, func, x, y, w, h)
            pto_len = len(temp_buf)
            pto_buf = "$%02d,%02d,%03d,%03d,%03d,%03d#\n" % (pto_len, func, x, y, w, h)
        else:
            temp_buf = "$%02d,%02d,%03d,%03d,%03d,%03d,%s#" % (pto_len, func, x, y, w, h, msg)
            pto_len = len(temp_buf)
            pto_buf = "$%02d,%02d,%03d,%03d,%03d,%03d,%s#\n" % (pto_len, func, x, y, w, h, msg)
        return pto_buf
    

    def package_message(self, func, msg, value=None):
        pto_len = 0
        if value is None:
            temp_buf = "$%02d,%02d,%s#" % (pto_len, func, msg)
            pto_len = len(temp_buf)
            pto_buf = "$%02d,%02d,%s#\n" % (pto_len, func, msg)
        else:
            temp_buf = "$%02d,%02d,%s,%03d#" % (pto_len, func, msg, value)
            pto_len = len(temp_buf)
            pto_buf = "$%02d,%02d,%s,%03d#\n" % (pto_len, func, msg, value)
        return pto_buf

    def package_msg_value(self, func, x, y, w, h, msg, value):
        pto_len = 0
        temp_buf = "$%02d,%02d,%03d,%03d,%03d,%03d,%s,%03d#" % (pto_len, func, x, y, w, h, msg, value)
        pto_len = len(temp_buf)
        pto_buf = "$%02d,%02d,%03d,%03d,%03d,%03d,%s,%03d#\n" % (pto_len, func, x, y, w, h, msg, value)
        return pto_buf
    
    def package_apriltag(self, func, x, y, w, h, tag_id, degrees):
        pto_len = 0
        temp_buf = "$%02d,%02d,%03d,%03d,%03d,%03d,%03d,%03d#" % (pto_len, func, x, y, w, h, tag_id, degrees)
        pto_len = len(temp_buf)
        pto_buf = "$%02d,%02d,%03d,%03d,%03d,%03d,%03d,%03d#\n" % (pto_len, func, x, y, w, h, tag_id, degrees)
        return pto_buf
    
    def package_licence(self, func, msg):
        pto_len = 0
        temp_buf = "$%02d,%02d,%s#" % (pto_len, func, msg)
        pto_len = len(temp_buf) + 2
        pto_buf = "$%02d,%02d,%s#\n" % (pto_len, func, msg)
        return pto_buf

    #########################################################################################################
    #########################################################################################################
    #########################################################################################################
    
    def get_color_data(self, x, y, w, h):
        func_id = self.ID_COLOR
        data = self.package_coord(func_id, int(x), int(y), int(w), int(h))
        return data
    
    def get_barcode_data(self, x, y, w, h, msg):
        func_id = self.ID_BARCODE
        data = self.package_coord(func_id, int(x), int(y), int(w), int(h), msg)
        return data
    
    def get_qrcode_data(self, x, y, w, h, msg):
        func_id = self.ID_QRCODE
        data = self.package_coord(func_id, int(x), int(y), int(w), int(h), msg)
        return data
    
    def get_apriltag_data(self, x, y, w, h, tag_id, degrees):
        func_id = self.ID_APRILTAG
        data = self.package_apriltag(func_id, int(x), int(y), int(w), int(h), int(tag_id), int(degrees))
        return data
    
    def get_dmcode_data(self, x, y, w, h, msg, degrees):
        func_id = self.ID_DMCODE
        data = self.package_msg_value(func_id, int(x), int(y), int(w), int(h), msg, int(degrees))
        return data

    def get_face_detect_data(self, x, y, w, h):
        func_id = self.ID_FACE_DETECT
        data = self.package_coord(func_id, int(x), int(y), int(w), int(h))
        return data
    
    def get_eye_gaze_data(self, start_x, start_y, end_x, end_y):
        func_id = self.ID_EYE_GAZE
        data = self.package_coord(func_id, int(start_x), int(start_y), int(end_x), int(end_y))
        return data
    
    def get_face_recoginiton_data(self, x, y, w, h, name, score):
        func_id = self.ID_FACE_RECOGNITION
        data = self.package_msg_value(func_id, int(x), int(y), int(w), int(h), name, int(float(score)*100))
        return data
    
    def get_person_detect_data(self, x, y, w, h):
        func_id = self.ID_PERSON_DETECT
        data = self.package_coord(func_id, int(x), int(y), int(w), int(h))
        return data

    def get_falldown_detect_data(self, x, y, w, h, msg, score):
        func_id = self.ID_FALLDOWN_DETECT
        data = self.package_msg_value(func_id, int(x), int(y), int(w), int(h), msg, int(float(score)*100))
        return data


    def get_hand_detect_data(self, x, y, w, h):
        func_id = self.ID_HAND_DETECT
        data = self.package_coord(func_id, int(x), int(y), int(w), int(h))
        return data


    def get_hand_gesture_data(self, msg):
        func_id = self.ID_HAND_GESTURE
        data = self.package_message(func_id, msg)
        return data

    def get_ocr_rec_data(self, msg):
        func_id = self.ID_OCR_REC
        data = self.package_message(func_id, msg)
        return data

    def get_object_detect_data(self, x, y, w, h, msg):
        func_id = self.ID_OBJECT_DETECT
        data = self.package_coord(func_id, int(x), int(y), int(w), int(h), msg)
        return data

    def get_nano_tracker_data(self, x, y, w, h):
        func_id = self.ID_NANO_TRACKER
        data = self.package_coord(func_id, int(x), int(y), int(w), int(h))
        return data

    def get_self_learning_data(self, category, score):
        func_id = self.ID_SELF_LEARNING
        data = self.package_message(func_id, category, int(float(score)*100))
        return data

    def get_licence_rec_data(self, msg):
        func_id = self.ID_LICENCE_REC
        data = self.package_licence(func_id, msg)
        return data

