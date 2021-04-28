from styx_msgs.msg import TrafficLight
import tensorflow as tf
import yaml
import rospy
import numpy as np
import os

# Changing the directory to the current directory of "light_classification", so that the script can find both
# the models saved.

current_dir = os.path.dirname(os.path.realpath(__file__))

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        
        # Loading the classification model
        os.chdir(current_dir)
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.is_site = self.config["is_site"]
        model_to_load = None
        
        # Loading different models based on the state values 
        # as there are two different models for site and sim. 
        if self.is_site:
            model_to_load = "classification_model_site.pb"
        else:
            model_to_load = "classification_model_sim.pb"
        
        print("Model Loaded")
        self.graph = tf.Graph()

        with self.graph.as_default():
            od_graph = tf.GraphDef()
            with tf.gfile.GFile(model_to_load, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph, name='')

            self.sess = tf.Session(graph=self.graph)
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        with self.graph.as_default():
            assert image.shape[1] == 800, "Wrong Width image"
            assert image.shape[0] == 600, "Wrong Height image"
            assert image.shape[2] == 3, "The image has to be color, Check Image color channels."
            if len(image.shape) < 4:
                expanded_img = np.expand_dims(image, axis=0)
                
            (scores, classes) = self.sess.run([self.scores, self.classes], feed_dict={self.image_tensor: expanded_img})

        classes = list(np.squeeze(classes))
        scores = np.squeeze(scores)
        
#         print(classes)
#         print(scores)
        
        # if the model can't detect anything
        if len(scores) == 0 or scores[0] < 0.1:
            return TrafficLight.UNKNOWN
        
        # get the results and return according state of the color
        else:
            # print("Did an inference, Returning proper state")
            result_class = classes[0]
            if result_class == 1:
                return TrafficLight.GREEN
            elif result_class == 2:
                return TrafficLight.RED
            elif result_class == 3:
                return TrafficLight.YELLOW
            
        return TrafficLight.UNKNOWN
