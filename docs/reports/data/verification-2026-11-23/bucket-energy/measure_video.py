"""Measure the original 1468 × 847 capture. The optional image marks changing pixels in red."""
import argparse
import cv2
import numpy as np

p = argparse.ArgumentParser()
p.add_argument("video")
p.add_argument("--image")
args = p.parse_args()
cap = cv2.VideoCapture(args.video)
ok, first = cap.read()
if not ok or first.shape[:2] != (847,1468):
    raise ValueError("The regions of interest require the original 1468 × 847 capture")
gray0 = cv2.cvtColor(first, cv2.COLOR_BGR2GRAY)
regions = {
    "upper": [(740,264),(852,213),(1009,201),(1061,384),(811,455)],
    "middle": [(773,549),(832,563),(979,568),(1050,545),(1027,590),(834,607),(777,590)],
    "lower": [(786,640),(1020,640),(989,811),(844,812)],
    "floor": [(380,280),(660,280),(660,800),(380,800)],
}
points, displacement = {}, {key:[] for key in regions}
for name, vertices in regions.items():
    mask = np.zeros(gray0.shape, np.uint8)
    cv2.fillPoly(mask,[np.array(vertices,dtype=np.int32)],255)
    points[name] = cv2.goodFeaturesToTrack(gray0,100,0.003,7,mask=mask)
high, low = first.copy(), first.copy()
frame, previous, changed, count = first, None, 0, 0
kernel = np.ones((5,5),np.uint8)
while ok:
    high,low = np.maximum(high,frame),np.minimum(low,frame)
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    for name, start in points.items():
        now, status, _ = cv2.calcOpticalFlowPyrLK(gray0,gray,start,None,winSize=(21,21),maxLevel=3)
        back, valid, _ = cv2.calcOpticalFlowPyrLK(gray,gray0,now,None,winSize=(21,21),maxLevel=3)
        keep = (status.ravel()>0) & (valid.ravel()>0) & (np.linalg.norm((back-start).reshape(-1,2),axis=1)<.65)
        if keep.sum() >= 4:
            displacement[name].append(np.median((now-start).reshape(-1,2)[keep],axis=0))
    b,g,r = cv2.split(frame[545:,670:1150])
    yellow = ((r>200)&(g>152)&(b<95)).astype(np.uint8)
    if previous is not None:
        new = np.sum(yellow * (1-cv2.dilate(previous,kernel)))
        lost = np.sum(previous * (1-cv2.dilate(yellow,kernel)))
        changed += int(max(new,lost)>8)
    previous = yellow
    count += 1
    ok,frame = cap.read()
print(f"frames={count} adjacent_pairs={count-1} changed_normal_lines={changed}")
for key in points:
    delta = np.asarray(displacement[key])
    print(f"{key}: features={len(points[key])} max_norm_of_median_shift_px={np.linalg.norm(delta,axis=1).max():.7g}")
if args.image:
    image = first[30:,368:].copy()
    span = (high.astype(int)-low.astype(int)).max(axis=2)
    image[span[30:,368:]>86] = (30,35,255)
    if not cv2.imwrite(args.image,image):
        raise IOError(args.image)
