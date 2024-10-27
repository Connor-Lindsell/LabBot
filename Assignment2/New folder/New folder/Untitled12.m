link(1) = Link([pi/2     0.1012      0.033        pi/2	0]);
link(2) = Link([0     0.0      0.155        0	0]);
link(3) = Link([0     0.0      0.1348        0	0]);
link(4) = Link([pi/2     0.019      0.0005        -pi/2	0]);
model = SerialLink(link,'name','RRRR');
model.teach([0 0 0 0])

