# Computer_Graphics
Our assignment for the Computer Graphics unit. We were tasked to develop a 3D rasteriser and raytracer in C++ using the SDL and GLM libraries to create a 10-second animation using a pre-given OBJ file. We were given a list of features to implement, listed below. We submitted three animation videos, the first in wireframe mode, the second rendered using our shaded triangle rasteriser, and the final using our raytracer (which has the most features). Our raytracer animation is displayed in the GIF below.

<img src="/CW-Hackspace/finished_videos/raytracer_animation.gif"></img>

## Indicative Marking Guidelines
- \# = not mandatory to move to next range
- Bold = done, italics = sort of

### 40s range
  - [x] **OBJ loading (geometry and materials)**
  - [x] **Wireframe rendering**
  - [x] **Flat shaded rasterising**
  - [x] **Moving camera (position only)**
  - [x] **Saving of PPM files**

### 50s range
- [x] **Changing camera orientation (via orientation matrix)**
- [x] **LookAt**
- [x] *Some form of generative geometry* \#
- [x] **Ambient lighting**
- [x] **Diffuse lighting (proximity and angle-of-incidence)**

### 60s range
- [x] **Hard Shadow**
- [x] **Gouraud shading**
- [x]  **Naive (non-perspective corrected) texture mapping**
- [x] *Simple animation (e.g. fly-through)*
- [x] **Some form of Physics (gravity, friction, bounce etc) \#**
- [x] **Basic culling (far-plane, near-plan, back-face)**
- [x] *Basic anti-aliasing* \#

### 70s range
- [x] **Phong Shading**
- [x] **Fairly “interesting” materials (e.g. mirrors)**
- [x] **Complex anti-aliasing**
- [x] **Perspective corrected texture mapping**
- [x] *“Fancy” animation*
- [x] *Soft shadow*
- [x] **Bump maps**
- [x] **Basic clipping (bounding box)**

### 80s range
- [ ] Environment maps
- [x] *Complex “interesting” materials (e.g. glass)*
- [ ] Line drawing optimisations (e.g. Bresenham)
- [x] **Advance clipping (frustum)**
- [ ] Photon maps and caustics
- [ ] “Intelligent” model simplification
