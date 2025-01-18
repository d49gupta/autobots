import cv2
import numpy as np

# Create a simple image
image = np.zeros((500, 500, 3), dtype=np.uint8)
cv2.putText(image, 'Hello, OpenCV!', (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

# Display the image
cv2.imshow("OpenCV Window", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
