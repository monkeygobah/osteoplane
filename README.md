# osteoplane

## Code repository for osteoplane- a tool to quantify the error in fibula free flaps (FFF)

### Works by two methods
1.  by registering the point clouds of individual segments of the FFF so that the *xz* plane becomes a shared reference plane. The delta between the proximal or distal angle 
of a segment relative to this plane is calculated, and the delta between pre and post op is calcualted and reported
2. The euler angles needed to rotate the post operative plane for any segment to become parallel to the planned plane is found. Euler angles then represent the x,y,z angular error from post-preop


### Usage

CT scans of planned and postoperative models must be selected. Then the user picks the points that define the planes proximally and distally on all segments using commands from Open3D. Following this, the pipeline runs. Best results found from analyuzing the same segment ~3 times to obtain an average of error 

### Example Outputs

<img width="577" alt="image" src="https://github.com/monkeygobah/osteoplane/assets/117255104/6376c968-ed2c-4b10-bb35-de9bbcd2ed39">

![image](https://github.com/monkeygobah/osteoplane/assets/117255104/4ab3598d-dbd3-48c8-94f6-8f91d6cf7f7d)

<img width="550" alt="image" src="https://github.com/monkeygobah/osteoplane/assets/117255104/8949c5f4-290f-4fd9-9430-627afc288624">

![image](https://github.com/monkeygobah/osteoplane/assets/117255104/e8b9b229-f850-4a1c-b837-9f73ccba15b3)


