import numpy as np

class CircularBuffer:
    def __init__(self, size, num_elements):
        self.size = size
        self.buffer = [None] * size
        self.index = 0
        self.is_full = False
        self.num_elements = num_elements

    def append(self, element):
        if not isinstance(element, np.ndarray) or element.shape != (self.num_elements,):
            raise ValueError("Element must be a numpy array of length " + str(self.num_elements))
        
        self.buffer[self.index] = element
        self.index = (self.index + 1) % self.size
        if self.index == 0:
            self.is_full = True

    def get_buffer(self):
        if not self.is_full:
            return self.buffer[:self.index]
        else:
            return self.buffer[self.index:] + self.buffer[:self.index]
    
    def get_mean(self):
        """
        Calculate the mean of the elements currently in the buffer.

        Returns:
            np.array: The mean of the elements in the buffer.
        """
        current_buffer = self.get_buffer()
        
        if not current_buffer:
            return np.zeros(self.num_elements)  # Return zeros if the buffer is empty
        
        return np.mean(current_buffer, axis=0)
    
    def negateQuaternion(self,quat): 
        if quat[3] < 0:
            quat = -1*quat
        return quat