This package uses a free SDF tool by Catlike Coding and a shader written by Brian Kehrer to convert PNGs into vectorized images for top tier smooth iconography, even when viewed at very close distances. 

How does this work? See:
https://steamcdn-a.akamaihd.net/apps/valve/2007/SIGGRAPH2007_AlphaTestedMagnification.pdf

How to use existing SDF icons:
1. They look blurry at first and THAT IS NORMAL! Apply the SDF PNG to your Protokit UI Panel or Image component. Then in the empty slot for a Material, insert the Unlit_SDF material included in this package. Done!

How to make your own vectorized icon out of a PNG:
1. Locate a PNG icon you want to vectorize. It must be 256px or larger to ensure nice smooth edges without wobbles. Sometimes you can get away with 128, but Brian recommends 256. 
2. Open the SDF Texture Generator window under Window -> SDF Texture Generator
3. Export your SDF PNG. It looks blurry, I know, but that's expected. trust the process
4. Adjust your SDF PNG in the inspector window to be a Single Sprite with mipmaps enabled
5. Stick your new SDF PNG sprite into a Protokit UI Panel or Image component. In the empty slot for a Material, insert the Unlit_SDF material included in this package. 
6. Appraise your new SDF image by zooming in super close and *chefs kiss* those edges man, so nice. good job!

Where do I get high resolution icons though?
1. ARDS icons here (you may need to ask Pol Pla for export permissions): https://www.figma.com/file/QglhlJOCnAc4ooAopACSOj/%F0%9F%9A%80-AR-Design-System---ARDS-for-Orion?type=design&node-id=13134-7589&mode=design&t=D0V7Eo0dlOL3nxXt-0
2. Oculus Icons here: https://www.internalfb.com/intern/assets/set/oculus_icons/10s-forward/size:24,variant:Filled/
3. Alternatively export icons directly from your designer's figma at x3 or x4