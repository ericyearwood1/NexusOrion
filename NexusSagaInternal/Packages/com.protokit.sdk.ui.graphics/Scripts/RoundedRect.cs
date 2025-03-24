using System;
using UnityEngine;
using OSIG.Tools.Layout;
using OSIG.Tools.Units;
using UnityEngine.Rendering;
using UnityEngine.UI;
using UnityEngine.Serialization;
#if UNITY_EDITOR
using OSIG.Tools.Utility;
using OSIG.Tools.Layout.Internals;
using UnityEditor;
using UnityEditor.EditorTools;
#endif

namespace ProtoKit.GraphicBase {
    [ExecuteAlways, AddComponentMenu("OSIG UI Core/Rounded Rect"), RequireComponent(typeof(CanvasRenderer))]
    public class RoundedRect : MaskableGraphic, IOCUnitsContextCache
#if OSIG_PANEL_DEPTH
    , IOCLayoutListener
#endif
    {
        #region PROPERTIES

        [FormerlySerializedAs("Width")] [SerializeField]
        protected OCValue _width;

        public OCValue Width {
            get { return _width; }
            set {
                _width = value;
                var layout = GetComponent<OCLayoutSize>();
                if (layout == null)
                    rectTransform.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal,
                        value.GetMeters(UnitsContext));
                else
                    layout.SetWidth(value);
            }
        }

        [FormerlySerializedAs("Height")] [SerializeField]
        protected OCValue _height;

        public OCValue Height {
            get { return _height; }
            set {
                _height = value;
                var layout = GetComponent<OCLayoutSize>();
                if (layout == null)
                    rectTransform.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, value.GetMeters(UnitsContext));
                else
                    layout.SetHeight(value);
            }
        }

        // corners
        [FormerlySerializedAs("Radius")] [SerializeField]
        protected OCValue _radius;

        public OCValue Radius {
            get { return _radius; }
            set { SetRadius(value); }
        }

        public bool IndependentCorners = false;

        [FormerlySerializedAs("RadiusTL")] [SerializeField]
        protected OCValue _radiusTL;

        public OCValue RadiusTL {
            get { return _radiusTL; }
            set { SetRadiusTL(value); }
        }

        [FormerlySerializedAs("RadiusTR")] [SerializeField]
        protected OCValue _radiusTR;

        public OCValue RadiusTR {
            get { return _radiusTR; }
            set { SetRadiusTR(value); }
        }

        [FormerlySerializedAs("RadiusBR")] [SerializeField]
        protected OCValue _radiusBR;

        public OCValue RadiusBR {
            get { return _radiusBR; }
            set { SetRadiusBR(value); }
        }

        [FormerlySerializedAs("RadiusBL")] [SerializeField]
        protected OCValue _radiusBL;

        public OCValue RadiusBL {
            get { return _radiusBL; }
            set { SetRadiusBL(value); }
        }

        // gradient is determined by the 3 values of colors
        [Range(1, 3)] public int ColorsNum = 1;

        [FormerlySerializedAs("ColorA")] [SerializeField]
        protected Color _colorA = Color.white;

        public Color ColorA {
            get { return _colorA; }
            set { SetColorA(value); }
        }

        [FormerlySerializedAs("ColorB")] [SerializeField]
        protected Color _colorB = Color.white;

        public Color ColorB {
            get { return _colorB; }
            set { SetColorB(value); }
        }

        [FormerlySerializedAs("ColorC")] [SerializeField]
        protected Color _colorC = Color.white;

        public Color ColorC {
            get { return _colorC; }
            set { SetColorC(value); }
        }

        [Range(0, 360)] public int GradientRotation = 0;

        // optional borders
        public enum BorderTypes {
            Inside = 0, // normal is inside
            Center,
            Outside,
        }

        [FormerlySerializedAs("BorderType")] [SerializeField]
        protected BorderTypes _borderType;

        public BorderTypes BorderType {
            get { return _borderType; }
            set { SetBorderType((int)value); }
        }

        [FormerlySerializedAs("Border")] [SerializeField]
        protected OCValue _border;

        public OCValue Border {
            get { return _border; }
            set { SetBorder(value); }
        }

        [FormerlySerializedAs("BorderColor")] [SerializeField]
        protected Color _borderColor = Color.black;

        public Color BorderColor {
            get { return _borderColor; }
            set { SetBorderColor(value); }
        }

        [SerializeField] protected OCValue _outlineOffset;

        public OCValue OutlineOffset {
            get { return _outlineOffset; }
            set { SetBorderOffset(value); }
        }


        /* GLOW */

        [FormerlySerializedAs("Border")] [SerializeField]
        protected OCValue _glowSpread;

        public OCValue GlowSpread {
            get { return _glowSpread; }
            set { SetGlowSpread(value); }
        }

        [FormerlySerializedAs("BorderColor")] [SerializeField]
        protected Color _glowColor = Color.black;

        public Color GlowColor {
            get { return _glowColor; }
            set { SetGlowColor(value); }
        }

        [SerializeField] protected OCValue _glowBlur;

        public OCValue GlowBlur {
            get { return _glowBlur; }
            set { SetGlowBlur(value); }
        }

        [SerializeField] protected OCValue _glowOffsetX;

        public OCValue GlowOffsetX {
            get { return _glowBlur; }
            set { SetGlowOffsetX(value); }
        }

        [SerializeField] protected OCValue _glowOffsetY;

        public OCValue GlowOffsetY {
            get { return _glowBlur; }
            set { SetGlowOffsetY(value); }
        }

        [FormerlySerializedAs("Image")] [SerializeField]
        protected Texture _image;

        public Texture Image {
            get { return _image; }
            set { SetImage(value); }
        }

        [SerializeField] protected Sprite _sprite;

        public Sprite Sprite {
            get { return _sprite; }
            set { SetImage(value); }
        }

        public enum ImageType {
            Sprite,
            Texture
        }

        public ImageType _imageType = ImageType.Sprite;

        public enum ImageFitMode {
            Fill,
            Fit,
            Stretch,
            Tile
        }

        public ImageFitMode _imageMode = ImageFitMode.Fill;

        [SerializeField]private OCValue _tileSizeX = new(0.1f, OCUnits.Meters);

        public OCValue TileSizeX {
            get => _tileSizeX;
            set => _tileSizeX = value;
        }

        [SerializeField]private OCValue _tileSizeY= new(0.1f, OCUnits.Meters);

        public OCValue TileSizeY {
            get => _tileSizeY;
            set => _tileSizeY = value;
        }

        #endregion

        #region SETTERS / GETTERS


        #region Dimensions

        /// <summary>
        /// Set width and height in px.
        /// </summary>
        /// <param name="size"></param>
        public void SetSize(Vector2 size) {
            SetWidth(size.x);
            SetHeight(size.y);
        }

        /// <summary>
        /// Set width in px.
        /// </summary>
        /// <param name="width"></param>
        public void SetWidth(float width) {
            SetWidth(width.AsPixels());
        }

        /// <summary>
        /// Set width in OCValue as string. e.g 10 px, 1 m, 20 deg.
        /// </summary>
        /// <param name="width"></param>
        public void SetWidth(string width) {
            SetWidth(OCValue.Parse(width));
        }

        /// <summary>
        /// Set width directly using OCValue.
        /// </summary>
        /// <param name="width"></param>
        public void SetWidth(OCValue width) {
            Width = width;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set height in px.
        /// </summary>
        /// <param name="height"></param>
        public void SetHeight(float height) {
            SetHeight(height.AsPixels());
        }

        /// <summary>
        /// Set height in OCValue as string. e.g 10 px, 1 m, 20 deg.
        /// </summary>
        /// <param name="height"></param>
        public void SetHeight(string height) {
            SetHeight(OCValue.Parse(height));
        }

        /// <summary>
        /// Set height directly using OCValue.
        /// </summary>
        /// <param name="height"></param>
        public void SetHeight(OCValue height) {
            Height = height;
            UpdateRepresentation();
        }

        #endregion

        #region Radius

        /// <summary>
        /// Set corner radius of all 4 corners in px.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadius(float radius) {
            SetRadius(radius.AsPixels());
        }

        /// <summary>
        /// Set corner radius of all 4 corners in OCValue as string. e.g 10 px, 1 m, 20 deg.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadius(string radius) {
            SetRadius(OCValue.Parse(radius));
        }

        /// <summary>
        /// Set radius directly using OCValue.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadius(OCValue radius) {
            _radius = radius;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set radius of top left corner in px.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusTL(float radius) {
            SetRadiusTL(radius.AsPixels());
        }

        /// <summary>
        /// Set radius of top left corner in OCValue as string. e.g 10 px, 1 m, 20 deg.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusTL(string radius) {
            SetRadiusTL(OCValue.Parse(radius));
        }

        /// <summary>
        /// Set radius of top left corner directly using OCValue.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusTL(OCValue radius) {
            IndependentCorners = true;
            _radiusTL = radius;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set radius of top right corner in px.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusTR(float radius) {
            SetRadiusTR(radius.AsPixels());
        }

        /// <summary>
        /// Set radius of top right corner in OCValue as string. e.g 10 px, 1 m, 20 deg.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusTR(string radius) {
            SetRadiusTR(OCValue.Parse(radius));
        }

        /// <summary>
        /// Set radius of top right corner directly using OCValue.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusTR(OCValue radius) {
            IndependentCorners = true;
            _radiusTR = radius;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set radius of bottom left corner in px.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusBL(float radius) {
            SetRadiusBL(radius.AsPixels());
        }

        /// <summary>
        /// Set radius of bottom left corner in OCValue as string. e.g 10 px, 1 m, 20 deg.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusBL(string radius) {
            SetRadiusBL(OCValue.Parse(radius));
        }

        /// <summary>
        /// Set radius of bottom left corner directly using OCValue.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusBL(OCValue radius) {
            IndependentCorners = true;
            _radiusBL = radius;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set radius of bottom right corner in px.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusBR(float radius) {
            SetRadiusBR(radius.AsPixels());
        }

        /// <summary>
        /// Set radius of bottom right corner in OCValue as string. e.g 10 px, 1 m, 20 deg.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusBR(string radius) {
            SetRadiusBR(OCValue.Parse(radius));
        }

        /// <summary>
        /// Set radius of bottom right corner directly using OCValue.
        /// </summary>
        /// <param name="radius"></param>
        public void SetRadiusBR(OCValue radius) {
            IndependentCorners = true;
            _radiusBR = radius;
            UpdateRepresentation();
        }

        #endregion

        #region Glow

        /// Set glow color.
        /// </summary>
        /// <param name="color"></param>
        public void SetGlowColor(Color color) {
            _glowColor = color;
            UpdateRepresentation();
        }

        /// Set glow spread as float.
        /// </summary>
        /// <param name="spread"></param>
        public void SetGlowSpread(float spread) {
            SetGlowSpread(spread.AsPixels());
        }
        /// Set spread spread as string - "10px" "1m"
        /// </summary>
        /// <param name="spread"></param>
        public void SetGlowSpread(string spread) {
            SetGlowSpread(OCValue.Parse(spread));
        }
        /// SSet spread blur as OCValue.
        /// </summary>
        /// <param name="blur"></param>
        public void SetGlowSpread(OCValue blur) {
            _glowSpread = blur;
            UpdateRepresentation();
        }

        /// Set glow blur as float.
        /// </summary>
        /// <param name="blur"></param>
        public void SetGlowBlur(float blur) {
           SetGlowBlur(blur.AsPixels());
        }
        /// Set glow blur as string - "10px" "1m"
        /// </summary>
        /// <param name="blur"></param>
        public void SetGlowBlur(string blur) {
            SetGlowBlur(OCValue.Parse(blur));
        }
        /// SSet glow blur as OCValue.
        /// </summary>
        /// <param name="blur"></param>
        public void SetGlowBlur(OCValue blur) {
            _glowBlur = blur;
            UpdateRepresentation();
        }
        /// Set glow offset X as float.
        /// </summary>
        /// <param name="offsetX"></param>
        public void SetGlowOffsetX(float offsetX) {
            SetGlowOffsetX(offsetX.AsPixels());
        }
        /// SSet glow offset X as OCValue.
        /// </summary>
        /// <param name="offsetX"></param>
        public void SetGlowOffsetX(string offsetX) {
            SetGlowOffsetX(OCValue.Parse(offsetX));
        }
        /// Set glow offset X as string - "10px" "1m"
        /// </summary>
        /// <param name="offsetX"></param>

        public void SetGlowOffsetX(OCValue offsetX) {
            _glowOffsetX = offsetX;
            UpdateRepresentation();
        }

        /// Set glow offset Y as float.
        /// </summary>
        /// <param name="offsetY"></param>
        public void SetGlowOffsetY(float offsetY) {
            SetGlowOffsetX(offsetY.AsPixels());
        }
        /// SSet glow offset Y as OCValue.
        /// </summary>
        /// <param name="offsetY"></param>
        public void SetGlowOffsetY(string offsetY) {
            SetGlowOffsetX(OCValue.Parse(offsetY));
        }
        /// Set glow offset Y as string - "10px" "1m"
        /// </summary>
        /// <param name="offsetY"></param>

        public void SetGlowOffsetY(OCValue offsetY) {
            _glowOffsetY = offsetY;
            UpdateRepresentation();
        }

        public void SetGlowOffset(Vector2 offset) {
            SetGlowOffsetX(offset.x);
            SetGlowOffsetY(offset.y);
        }

        public void SetGlowOffset(OCValue x, OCValue y) {
            SetGlowOffsetX(x);
            SetGlowOffsetY(y);
        }



        #endregion

        #region Colors

        /// <summary>
        /// Set first gradient color. This is also the main color.
        /// </summary>
        /// <param name="color"></param>
        public void SetColorA(Color color) {
            _colorA = color;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set first gradient color. This is also the main color.
        /// </summary>
        /// <param name="color">
        /// Hex color value. #RRGGBBAA
        /// </param>
        public void SetColorA(string color) {
            Color v;
            var parsed = ColorUtility.TryParseHtmlString(color, out v);
            if (parsed) {
                _colorA = v;
            }
            else {
                throw new Exception($@"Supplied color ""{color}"" could not be parsed.");
            }

            UpdateRepresentation();
        }

        /// <summary>
        /// Set second gradient color.
        /// </summary>
        /// <param name="color"></param>
        public void SetColorB(Color color) {
            _colorB = color;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set second gradient color.
        /// </summary>
        /// <param name="color">
        /// Hex color value. #RRGGBBAA
        /// </param>
        public void SetColorB(string color) {
            Color v;
            var parsed = ColorUtility.TryParseHtmlString(color, out v);
            if (parsed) {
                _colorB = v;
            }
            else {
                throw new Exception($@"Supplied color ""{color}"" could not be parsed.");
            }

            UpdateRepresentation();
        }

        /// <summary>
        /// Set third gradient color.
        /// </summary>
        /// <param name="color"></param>
        public void SetColorC(Color color) {
            _colorC = color;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set third gradient color.
        /// </summary>
        /// <param name="color">
        /// Hex color value. #RRGGBBAA
        /// </param>
        public void SetColorC(string color) {
            Color v;
            var parsed = ColorUtility.TryParseHtmlString(color, out v);
            if (parsed) {
                _colorC = v;
            }
            else {
                throw new Exception($@"Supplied color ""{color}"" could not be parsed.");
            }

            UpdateRepresentation();
        }

        /// <summary>
        /// Set background image.
        /// </summary>
        /// <param name="img"></param>
        public void SetImage(Texture img) {
            _imageType = ImageType.Texture;
            _image = img;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set background image.
        /// </summary>
        /// <param name="img"></param>
        public void SetImage(Sprite img) {
            _imageType = ImageType.Sprite;
            _sprite = img;
            UpdateRepresentation();
        }

        #endregion

        #region Borders

        /// <summary>
        /// Set border size in px.
        /// </summary>
        /// <param name="border"></param>
        public void SetBorder(float border) {
            SetBorder(border.AsPixels());
        }

        /// <summary>
        /// Set border size in OCValue as string. e.g 10 px, 1 m, 20 deg.
        /// </summary>
        /// <param name="border"></param>
        public void SetBorder(string border) {
            SetBorder(OCValue.Parse(border));
        }

        /// <summary>
        /// Set border size directly using OCValue.
        /// </summary>
        /// <param name="border"></param>
        public void SetBorder(OCValue border) {
            _border = border;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set border type. 1 - inset, 2 - halfset, 3 - outset.
        /// </summary>
        /// <param name="type"></param>
        public void SetBorderType(int type) {
            if (type < 0) type = 0;
            else if (type > 2) type = 2;
            _borderType = (BorderTypes)type;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set border type. (I)inset, (H)halfset, (O)outset.
        /// </summary>
        /// <param name="type"></param>
        public void SetBorderType(string type) {
            _borderType = ParseBorderType(type);
            UpdateRepresentation();
        }

        /// <summary>
        /// Set border color.
        /// </summary>
        /// <param name="color"></param>
        public void SetBorderColor(Color color) {
            _borderColor = color;
            UpdateRepresentation();
        }

        /// <summary>
        /// Set border color.
        /// </summary>
        /// <param name="color">
        /// Hex color value. #RRGGBBAA
        /// </param>
        public void SetBorderColor(string color) {
            Color v;
            var parsed = ColorUtility.TryParseHtmlString(color, out v);
            if (parsed) {
                _borderColor = v;
            }
            else {
                throw new Exception($@"Supplied color ""{color}"" could not be parsed.");
            }

            UpdateRepresentation();
        }

        /// <summary>
        /// Set border offset in OCValue as string. e.g 10 px, 1 m, 20 deg.
        /// </summary>
        /// <param name="border"></param>
        public void SetBorderOffset(string outline) {
            SetBorderOffset(OCValue.Parse(outline));
        }

        /// <summary>
        /// Set border offset directly using OCValue.
        /// </summary>
        /// <param name="border"></param>
        public void SetBorderOffset(OCValue outline) {
            _outlineOffset = outline;
            UpdateRepresentation();
        }

        #endregion

        private BorderTypes ParseBorderType(string type) {
            var s = type.ToLower();
            if (s.CompareTo("outset") == 0) {
                return BorderTypes.Outside;
            }
            else if (s.CompareTo("halfset") == 0) {
                return BorderTypes.Center;
            }
            else if (s.CompareTo("inset") == 0) {
                return BorderTypes.Inside;
            }
            else {
                throw new Exception(
                    $@"Supplied border type ""{type}"" could not be parsed. Must be inset, halfset, or outset.");
            }
        }

        #endregion

        #region PUBLIC API

        public void SetMaterial(Material mat) {
            material = mat;
        }

        public void RestoreDefaultMaterial() {
            _cachedMaterial = null;
            SetMaterial(DefMaterial);
        }

        public Material DefMaterial {
            get {
                if (_cachedMaterial == null)
                    _cachedMaterial = Resources.Load("Materials/RoundedRect-Default") as Material;
                return _cachedMaterial;
            }
        }

        public override Texture mainTexture {
            get {
                if (_imageType == ImageType.Texture) return _image == null ? s_WhiteTexture : _image;
                else return _sprite == null ? s_WhiteTexture : _sprite.texture;
            }
        }

        public override Material defaultMaterial => DefMaterial;

        public override Material materialForRendering {
            get {
                GetComponents(typeof(IMaterialModifier), ComponentsListPool.ComponentsList);

                var modifiedMaterial = material;
                for (var i = 0; i < ComponentsListPool.ComponentsList.Count; i++) {
                    modifiedMaterial =
                        (ComponentsListPool.ComponentsList[i] as IMaterialModifier).GetModifiedMaterial(
                            modifiedMaterial);
                }

                //Gotcha here.  OCPanel is a MaskableGraphic which is a IMaterialModifier, so the above loop will always modify the material.
                //Unfortunately maskable graphics make copies of materials and never update the materials unless they are cleared and re-built.
                //This is marked as by-design here: https://issuetracker.unity3d.com/issues/changes-to-the-ui-image-material-color-doesnt-show-up-when-the-image-is-masked
                //We workaround this by updating the material _again_ after the material has been modified.  We need to modify before and
                //after so that any copies that are made curing the material modification process have valid data as well.
                UpdateMaterialProperties();

                return modifiedMaterial;
            }
        }

        public OCUnitsContext.Native UnitsContext {
            get {
                _cachedContext.UpdateIfDirty(gameObject);
                return _cachedContext;
            }
        }

        public override void SetNativeSize() {
            switch (_imageType) {
                case ImageType.Sprite:
                    if (_sprite != null) {
                        SetSize(_sprite.rect.size);
                    }

                    break;
                case ImageType.Texture:
                    if (_image != null) {
                        SetSize(new Vector2(_image.width, _image.height));
                    }

                    break;
            }
        }

        public virtual void UpdateProperties() {
            UpdateRepresentation();
        }

        public virtual void UpdateRepresentation() {
            SetMaterialDirty();
            SetVerticesDirty();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        #endregion

        #region UNITY MESSAGES

#if UNITY_EDITOR
        protected override void OnValidate() {
            base.OnValidate();

            SetVerticesDirty();
        }

        protected override void Reset() {
            base.Reset();
            Init();
        }


#endif

        protected override void Awake() {
            base.Awake();
        }

        protected override void OnRectTransformDimensionsChange() {
            base.OnRectTransformDimensionsChange();

            var rt = GetComponent<RectTransform>();

            _width.SetFromMeters(UnitsContext, rt.rect.width);
            _height.SetFromMeters(UnitsContext, rt.rect.height);

            SetMaterialDirty();
        }

        protected override void OnTransformParentChanged() {
            base.OnTransformParentChanged();
            _cachedContext.Dirty();
            UpdateParentCanvas();
        }

        #endregion

        #region IMPLEMENTATION

        public Material _cachedMaterial;
        [SerializeField] private OCUnitsContext.Native _cachedContext;


        protected override void OnEnable() {
            base.OnEnable();
            Init();
        }

        protected virtual void Init() {
            workerMesh.MarkDynamic();
            useLegacyMeshGeneration = true;
            UpdateProperties();
            OnRectTransformDimensionsChange();
            UpdateParentCanvas();
            _cachedContext.Dirty();
        }

        private static bool IsAlphaCorrectingGammaEnabled() {
#if PROTOKIT_UI_GAMMA_CORRECTED_ALPHA
            return true;
#else
            return false;
#endif
        }
#if UNITY_EDITOR
        [InitializeOnLoadMethod]
#endif
        [RuntimeInitializeOnLoadMethod]
        private static void SetupAlphaCorrectingGlobalShaderKeyword() {

            Shader.SetKeyword(GlobalKeyword.Create("PROTOKIT_UI_GAMMA_CORRECTED_ALPHA"), IsAlphaCorrectingGammaEnabled() && QualitySettings.activeColorSpace == ColorSpace.Linear);
        }

        protected virtual void UpdateMaterialProperties() {
        }


        private void UpdateParentCanvas() {
            if (canvas != null) {
                canvas.additionalShaderChannels |=
                    AdditionalCanvasShaderChannels.TexCoord2 | AdditionalCanvasShaderChannels.TexCoord3;
            }
        }


        void IOCUnitsContextCache.OnUnitsContextDirtied() {
            _cachedContext.Dirty();
        }

#if OSIG_PANEL_DEPTH && UNITY_EDITOR
    // This method is not called unless the experimental extruded panel is enabled in the Preferences
    void IOCLayoutListener.OnApplyLayout(IOCLayoutComponent layoutComponent) {
        if (CommonComponentsPreferences.EnabledPanelDepth) {
            if (cachedExtrude != layoutComponent.CurrentSize.z) {
                cachedExtrude = layoutComponent.CurrentSize.z;
                OnRectTransformDimensionsChange();
            }
        }
    }
#endif

        #endregion

        #region MESH GENERATION

        protected override void OnPopulateMesh(VertexHelper vh) {
            vh.Clear();
            if (_width <= 0 || _height <= 0) {
                return;
            }

            var rect = GetPixelAdjustedRect();
            var imageUvs = CalculateUVRect(rect);

            //Adjust the mesh rect and image uvs to account for the edge mode
            //We need to adjust the uvs too to keep the image at the same size
            float borderRawValue = 0f;
            if (_border.RawValue > 0) {
                borderRawValue = _border.GetMeters(UnitsContext);
                Vector2 imageUvsEdgeAdjustment =
                    Vector3.Scale(new Vector2(borderRawValue / rect.width, borderRawValue / rect.height),
                        imageUvs.size);
                switch (BorderType) {
                    case BorderTypes.Center:
                        imageUvs.position -= imageUvsEdgeAdjustment * 0.5f;
                        imageUvs.size += imageUvsEdgeAdjustment;

                        rect.position -= Vector2.one * borderRawValue * 0.5f;
                        rect.size += Vector2.one * borderRawValue;
                        break;
                    case BorderTypes.Outside:
                        imageUvs.position -= imageUvsEdgeAdjustment;
                        imageUvs.size += imageUvsEdgeAdjustment * 2;

                        rect.position -= Vector2.one * borderRawValue;
                        rect.size += Vector2.one * borderRawValue * 2;
                        break;
                }
            }

            float maxRadius = Mathf.Min(rect.width / 2, rect.height / 2);
            float clampedEdge = borderRawValue;

            float defaultRadiusM = _radius.GetMeters(UnitsContext);
            float radiusMetersTL = IndependentCorners ? _radiusTL.GetMeters(UnitsContext) : defaultRadiusM;
            float radiusMetersTR = IndependentCorners ? _radiusTR.GetMeters(UnitsContext) : defaultRadiusM;
            float radiusMetersBL = IndependentCorners ? _radiusBL.GetMeters(UnitsContext) : defaultRadiusM;
            float radiusMetersBR = IndependentCorners ? _radiusBR.GetMeters(UnitsContext) : defaultRadiusM;

            //We build the mesh out of 4 quads instead of just 1.  We define this helper to allow us to
            //build each quad one at a time with the right settings
            void AddPanelQuad(Rect panelRect, float radius,float innerRadius, Color color, float uMin, float uMax, float vMin, float vMax,
                bool cornerXSign, bool cornerYSign, bool isBorderMesh,  bool isGlowMesh, float extrusion) {
                var currentVertCount = vh.currentVertCount;

                if ((GradientRotation / 90) % 2 == 0) {
                    vh.AddTriangle(currentVertCount, currentVertCount + 3, currentVertCount + 1);
                    vh.AddTriangle(currentVertCount, currentVertCount + 2, currentVertCount + 3);
                }
                else {
                    vh.AddTriangle(currentVertCount + 1, currentVertCount, currentVertCount + 2);
                    vh.AddTriangle(currentVertCount + 1, currentVertCount + 2, currentVertCount + 3);
                }


                if ( isGlowMesh ) {
                    radius += extrusion;
                }

                float clampedOuterRadius = Mathf.Min(maxRadius, radius);


                Vector2 rectMin = panelRect.min;
                Vector2 rectMax = panelRect.max;

                //Calculate the center of the curvature for this corner in local space of the rect
                Vector2 curvatureCenter = new Vector2(
                    cornerXSign ? rectMax.x - clampedOuterRadius : rectMin.x + clampedOuterRadius,
                    cornerYSign ? rectMax.y - clampedOuterRadius : rectMin.y + clampedOuterRadius);

                Vector2 toCenter;
                UIVertex v = default;

                v.uv2 = new Vector2(clampedOuterRadius - innerRadius, clampedOuterRadius);

                for (int y = 0; y < 2; y++) {
                    for (int x = 0; x < 2; x++) {
                        //First calculate default rect-space uvs, which are 0-1
                        v.uv0.x = Mathf.Lerp(uMin, uMax, x);
                        v.uv0.y = Mathf.Lerp(vMin, vMax, y);



                        //Calculate vertex position based on these uvs and the graphic rect
                        v.position.x = Mathf.Lerp(rectMin.x, rectMax.x, v.uv0.x);
                        v.position.y = Mathf.Lerp(rectMin.y, rectMax.y, v.uv0.y);

                        //Calculate the "center" of the corner curvature in the correct coordinate space
                        toCenter.x = cornerXSign ? v.position.x - curvatureCenter.x : curvatureCenter.x - v.position.x;
                        toCenter.y = cornerYSign ? v.position.y - curvatureCenter.y : curvatureCenter.y - v.position.y;

                        v.uv2 = new Vector4(clampedOuterRadius - innerRadius, clampedOuterRadius, toCenter.x,toCenter.y);

                        if (isBorderMesh||isGlowMesh) {
                            v.color = color;
                        }
                        else {
                            v.color = EvalGradient(v.uv0) * color;
                        }

                        //Finally remap from 0-1 uvs to image uvs
                        v.uv0.x = Mathf.Lerp(imageUvs.min.x, imageUvs.max.x, v.uv0.x);
                        v.uv0.y = Mathf.Lerp(imageUvs.min.y, imageUvs.max.y, v.uv0.y);

                        if (isBorderMesh) {
                            v.uv0.x = -101;
                        }else if (isGlowMesh) {
                            v.uv0.x = -201;
                        }


                        vh.AddVert(v);
                    }
                }
            }

            void AddPanelMesh(float extrude, Color color, bool isBorderMesh, bool isGlowMesh , Vector2 offset) {
                Rect panelRect = rect;
                panelRect.position -= Vector2.one * extrude - offset;
                panelRect.size += 2 * Vector2.one * extrude;

                var innerRadius = isGlowMesh ? _glowBlur.GetMeters(UnitsContext) : clampedEdge;

                AddPanelQuad(panelRect, radiusMetersTL,innerRadius, color,
                    0.0f, 0.5f, 0.5f, 1.0f, false, true, isBorderMesh, isGlowMesh,extrude);
                AddPanelQuad(panelRect, radiusMetersTR,innerRadius, color,
                    0.5f, 1.0f, 0.5f, 1.0f, true, true, isBorderMesh, isGlowMesh,extrude);
                AddPanelQuad(panelRect, radiusMetersBL, innerRadius,color,
                    0.0f, 0.5f, 0.0f, 0.5f, false, false, isBorderMesh, isGlowMesh,extrude);
                AddPanelQuad(panelRect, radiusMetersBR,innerRadius, color,
                    0.5f, 1.0f, 0.0f, 0.5f, true, false, isBorderMesh, isGlowMesh,extrude);
            }

            //Glow
            if (_glowSpread.GetMeters(UnitsContext) >= Mathf.Epsilon || _glowBlur.GetMeters(UnitsContext) >= Mathf.Epsilon ) {
                AddPanelMesh(_glowSpread.GetMeters(UnitsContext)+_glowBlur.GetMeters(UnitsContext)*0.5f, _glowColor, false, true, new Vector2(_glowOffsetX.GetMeters(UnitsContext),-_glowOffsetY.GetMeters(UnitsContext)));
            }


            //fill
            //We pass color white to support gradient further down in the process
            AddPanelMesh(0, Color.white, false, false, Vector2.zero);


            //Border
            if (clampedEdge >= Mathf.Epsilon) {
                AddPanelMesh(_outlineOffset.GetMeters(UnitsContext), _borderColor, true, false, Vector2.zero);
            }
        }

        /// <summary>
        /// Calculates the UV rect for the texture we will be displaying.  This method accounts for:
        ///  - Sprite atlasing.  It will return the 'windowed' UV rect that only represents the part of the texture containing the sprite.
        ///  - Fit mode. It will adjust the uv rect to account for the graphic being set to fit or fill mode.
        ///
        /// The resulting UV rect should be maped 1-to-1 to the resulting graphic.  The four corners of the visual graphic should return
        /// the 4 uv values returned by the resulting uv rectangle.
        /// </summary>
        private Rect CalculateUVRect(Rect pixelAdjustedRect) {
            //First calculate both the source image uvs, as well as the texel dimensions
            //based on whether or not the graphic is set to sprite or texture mode
            Rect sourceImageUvs;
            Vector2 sourceImageTexels;
            if (_imageType == ImageType.Sprite && _sprite != null) {
                sourceImageTexels = _sprite.rect.size;

                sourceImageUvs = _sprite.rect;
                sourceImageUvs.x /= _sprite.texture.width;
                sourceImageUvs.y /= _sprite.texture.height;
                sourceImageUvs.width /= _sprite.texture.width;
                sourceImageUvs.height /= _sprite.texture.height;
            }
            else {
                sourceImageTexels = new Vector2(mainTexture.width, mainTexture.height);
                sourceImageUvs = new Rect(0, 0, 1, 1);
            }

            float rectRatioX = pixelAdjustedRect.width / pixelAdjustedRect.height;
            float rectRatioY = pixelAdjustedRect.height / pixelAdjustedRect.width;
            float texelRatioX = sourceImageTexels.x / sourceImageTexels.y;
            float texelRatioY = sourceImageTexels.y / sourceImageTexels.x;

            //Represents whether or not the image has more empty space on the sides compared to the top/bottom when
            //placed completely inside of the graphic rectangle.
            bool moreEmptySpaceOnSides = (sourceImageTexels.y / pixelAdjustedRect.height) >
                                         (sourceImageTexels.x / pixelAdjustedRect.width);

            Rect imageUvs;

            switch (_imageMode) {

                case ImageFitMode.Fill:
                    Vector2 fillSize = moreEmptySpaceOnSides
                        ? new Vector2(1f, texelRatioX * rectRatioY)
                        : new Vector2(texelRatioY * rectRatioX, 1f);
                    fillSize *= sourceImageUvs.size;
                    imageUvs = new Rect(sourceImageUvs.center - fillSize * 0.5f, fillSize);
                    break;
                case ImageFitMode.Fit:
                    Vector2 fitSize = moreEmptySpaceOnSides
                        ? new Vector2(texelRatioY * rectRatioX, 1f)
                        : new Vector2(1f, texelRatioX * rectRatioY);
                    fitSize *= sourceImageUvs.size;
                    imageUvs = new Rect(sourceImageUvs.center - fitSize * 0.5f, fitSize);
                    break;
                case ImageFitMode.Tile:
                    Vector2 tileSize = new Vector2(1 /_tileSizeX.GetMeters(UnitsContext)* _width.GetMeters(UnitsContext), 1 /_tileSizeY.GetMeters(UnitsContext)*_height.GetMeters(UnitsContext));
                    imageUvs = new Rect(sourceImageUvs.center - tileSize * 0.5f, tileSize);

                    break;
                default:
                    imageUvs = sourceImageUvs;
                    break;
            }
            return imageUvs;
        }

        private Color EvalGradient(float t) {
            switch (ColorsNum) {
                default:
                case 1:
                    return _colorA;
                case 2:
                    return Color.Lerp(_colorA, _colorB, t);
                case 3:
                    if (t >= 0.5f) {
                        return Color.Lerp(_colorB, _colorC, t * 2 - 1);
                    }
                    else {
                        return Color.Lerp(_colorA, _colorB, t * 2);
                    }
            }
        }

        private Color EvalGradient(Vector2 uv) {
            var angleRad = Mathf.Deg2Rad * GradientRotation;
            var uvAroundCenter = uv - new Vector2(0.5f, 0.5f);

            Vector2 rotatedUv;
            rotatedUv.x = uvAroundCenter.x * Mathf.Cos(angleRad) - uvAroundCenter.y * Mathf.Sin(angleRad);
            rotatedUv.y = uvAroundCenter.y * Mathf.Cos(angleRad) + uvAroundCenter.x * Mathf.Sin(angleRad);

            Vector2 normalizedUv = rotatedUv + new Vector2(0.5f, 0.5f);

            return EvalGradient(normalizedUv.x);
        }

        #endregion

        #region EDITOR

#if UNITY_EDITOR
        [CanEditMultipleObjects]
        [CustomEditor(typeof(RoundedRect), true)]
        public class RoundedRectEditor : EditorBase {
            private RoundedRect _roundedRect;

            // common settings control flags
            public bool DisableShape = false;
            public bool DisableColors = false;

            // use extra settings to expose referenced children
            protected bool drawExtraSettings = false;
            protected bool showExtraSettings = false;

            //private bool showExtraSettings = false;
            protected bool showSizeSettings = true;
            protected bool showRadiusSettings = true;
            protected bool showColorSettings = true;
            protected bool showBorderSettings = true;
            protected bool showGlowSettings = true;

            protected bool showScriptLabel = true;
            protected bool showAddDistance = true;
            protected bool showInactive = true;
            protected bool showSizesection = true;

            private Gradient gradient;
            private Color cachedColorA;
            private Color cachedColorB;
            private Color cachedColorC;
            private int cachedNumColors;


            private SerializedProperty widthProp;

            private SerializedProperty heightProp;

            // corners
            private SerializedProperty independentCornersProp;
            private SerializedProperty radiusProp;
            private SerializedProperty radiusTLProp;
            private SerializedProperty radiusTRProp;
            private SerializedProperty radiusBRProp;

            private SerializedProperty radiusBLProp;

            // colors
            private SerializedProperty colorsNumProp;
            private SerializedProperty colorAProp;
            private SerializedProperty colorBProp;
            private SerializedProperty colorCProp;

            private SerializedProperty gradientRotationProp;

            // border
            private SerializedProperty borderProp;
            private SerializedProperty borderColorProp;
            private SerializedProperty borderTypeProp;
            private SerializedProperty outlineOffsetProp;

            // border
            private SerializedProperty glowColorProp;
            private SerializedProperty glowSpreadProp;
            private SerializedProperty glowBlurProp;
            private SerializedProperty glowOffsetXProp;
            private SerializedProperty glowOffsetYProp;

            // image
            private SerializedProperty imageType;
            private SerializedProperty imageMode;
            private SerializedProperty spriteProp;
            private SerializedProperty imageProp;
            private SerializedProperty materialProp;
            private SerializedProperty tileXProp;
            private SerializedProperty tileYProp;



            protected override void OnEnable() {
                _roundedRect = (RoundedRect)target;
                gradient = new Gradient();

                widthProp = serializedObject.FindProperty(nameof(_width));
                heightProp = serializedObject.FindProperty(nameof(_height));
                independentCornersProp = serializedObject.FindProperty(nameof(RoundedRect.IndependentCorners));
                radiusProp = serializedObject.FindProperty(nameof(RoundedRect._radius));
                radiusTLProp = serializedObject.FindProperty(nameof(RoundedRect._radiusTL));
                radiusTRProp = serializedObject.FindProperty(nameof(RoundedRect._radiusTR));
                radiusBRProp = serializedObject.FindProperty(nameof(RoundedRect._radiusBR));
                radiusBLProp = serializedObject.FindProperty(nameof(RoundedRect._radiusBL));
                colorsNumProp = serializedObject.FindProperty(nameof(RoundedRect.ColorsNum));
                colorAProp = serializedObject.FindProperty(nameof(RoundedRect._colorA));
                colorBProp = serializedObject.FindProperty(nameof(RoundedRect._colorB));
                colorCProp = serializedObject.FindProperty(nameof(RoundedRect._colorC));
                gradientRotationProp = serializedObject.FindProperty(nameof(RoundedRect.GradientRotation));
                borderProp = serializedObject.FindProperty(nameof(RoundedRect._border));
                borderColorProp = serializedObject.FindProperty(nameof(RoundedRect._borderColor));
                borderTypeProp = serializedObject.FindProperty(nameof(RoundedRect._borderType));
                outlineOffsetProp = serializedObject.FindProperty(nameof(RoundedRect._outlineOffset));
                glowBlurProp = serializedObject.FindProperty(nameof(RoundedRect._glowBlur));
                glowSpreadProp = serializedObject.FindProperty(nameof(RoundedRect._glowSpread));
                glowColorProp = serializedObject.FindProperty(nameof(RoundedRect._glowColor));
                glowOffsetXProp = serializedObject.FindProperty(nameof(RoundedRect._glowOffsetX));
                glowOffsetYProp = serializedObject.FindProperty(nameof(RoundedRect._glowOffsetY));
                imageType = serializedObject.FindProperty(nameof(RoundedRect._imageType));
                imageMode = serializedObject.FindProperty(nameof(RoundedRect._imageMode));
                spriteProp = serializedObject.FindProperty(nameof(RoundedRect._sprite));
                imageProp = serializedObject.FindProperty(nameof(RoundedRect._image));
                materialProp = serializedObject.FindProperty(nameof(RoundedRect.m_Material));
                tileXProp = serializedObject.FindProperty(nameof(RoundedRect._tileSizeX));
                tileYProp = serializedObject.FindProperty(nameof(RoundedRect._tileSizeY));

                foreach (var t in targets) {
                    if ((t as RoundedRect).GetComponent<IOCLayoutComponent>() != null) {
                        //DisableShape = true;
                        showSizeSettings = false;
                        showSizesection = false;
                    }
                }
            }

            protected override void OnDisable() {
                _roundedRect = null;
                widthProp.Dispose();
                heightProp.Dispose();
                independentCornersProp.Dispose();
                radiusProp.Dispose();
                radiusTLProp.Dispose();
                radiusTRProp.Dispose();
                radiusBRProp.Dispose();
                radiusBLProp.Dispose();
                colorsNumProp.Dispose();
                colorAProp.Dispose();
                colorBProp.Dispose();
                colorCProp.Dispose();
                gradientRotationProp.Dispose();
                borderProp.Dispose();
                borderColorProp.Dispose();
                borderTypeProp.Dispose();
                outlineOffsetProp.Dispose();
                glowBlurProp.Dispose();
                glowSpreadProp.Dispose();
                glowColorProp.Dispose();
                glowOffsetXProp.Dispose();
                glowOffsetYProp.Dispose();
                imageType.Dispose();
                imageMode.Dispose();
                spriteProp.Dispose();
                imageProp.Dispose();
                tileXProp.Dispose();
                tileYProp.Dispose();
            }

            public override void OnInspectorGUI() {
                var defaultLabelWidth = EditorGUIUtility.labelWidth;
                var labelWidth = 110;
                var guiColor = GUI.color;

                if (showAddDistance) {
                }

                if (showScriptLabel) {
                    DrawScriptHeader();
                    //DrawAppearanceHeader(labelWidth);
                }

                //check if the OC Layout Size script was attached or not

                EditorGUI.BeginChangeCheck();
                {
                    using (new GUILayout.VerticalScope()) {
                        EditorGUIUtility.hierarchyMode = false;

                        //Size setting
                        if (showSizesection) {
                            using (new EditorGUILayout.HorizontalScope(EditorStyles.helpBox)) {
                                showSizeSettings = EditorGUILayout.Foldout(showSizeSettings, "Size", true);
                            }
                        }

                        if (showSizeSettings) {
                            {
                                using (new GUILayout.VerticalScope()) {
                                    EditorGUIUtility.labelWidth = labelWidth;

                                    using (var check = new EditorGUI.ChangeCheckScope()) {
                                        EditorGUILayout.PropertyField(widthProp, new GUIContent("Width"));
                                        EditorGUILayout.PropertyField(heightProp, new GUIContent("Height"));

                                        if (check.changed) {
                                            //need to apply properties before changes are reflected in the properties themselves
                                            serializedObject.ApplyModifiedProperties();

                                            foreach (var t in targets) {
                                                Undo.RecordObject(t, "Set Size");
                                                var panel = t as RoundedRect;
                                                var rt = panel.rectTransform;
                                                rt.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal,
                                                    panel.Width.GetMeters(panel.UnitsContext));
                                                rt.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical,
                                                    panel.Height.GetMeters(panel.UnitsContext));
                                            }
                                        }
                                    }
                                }
                            }

                            EditorGUILayout.Space();
                        }

                        // reset
                        EditorGUIUtility.labelWidth = defaultLabelWidth;

                        //color setting
                        using (new EditorGUILayout.HorizontalScope(EditorStyles.helpBox)) {
                            showColorSettings = EditorGUILayout.Foldout(showColorSettings, "Color", true);
                        }

                        if (showColorSettings) {
                            using (new GUILayout.VerticalScope()) {
                                using (new GUILayout.HorizontalScope()) {
                                    // redraw gradient if needed
                                    if (cachedColorA != colorAProp.colorValue ||
                                        cachedColorB != colorBProp.colorValue ||
                                        cachedColorC != colorCProp.colorValue ||
                                        cachedNumColors != colorsNumProp.intValue) {
                                        gradient = new Gradient();
                                        switch (colorsNumProp.intValue) {
                                            case 1:
                                                gradient.colorKeys = new GradientColorKey[2] {
                                                    new GradientColorKey(colorAProp.colorValue, 0),
                                                    new GradientColorKey(colorAProp.colorValue, 1)
                                                };
                                                break;
                                            case 2:
                                                gradient.colorKeys = new GradientColorKey[2] {
                                                    new GradientColorKey(colorAProp.colorValue, 0),
                                                    new GradientColorKey(colorBProp.colorValue, 1)
                                                };
                                                break;
                                            case 3:
                                                gradient.colorKeys = new GradientColorKey[3] {
                                                    new GradientColorKey(colorAProp.colorValue, 0),
                                                    new GradientColorKey(colorBProp.colorValue, 0.5f),
                                                    new GradientColorKey(colorCProp.colorValue, 1)
                                                };
                                                break;
                                        }

                                        cachedColorA = colorAProp.colorValue;
                                        cachedColorB = colorBProp.colorValue;
                                        cachedColorC = colorCProp.colorValue;
                                        cachedNumColors = colorsNumProp.intValue;
                                    }
                                }

                                using (new EditorGUI.DisabledScope(DisableColors)) {
                                    using (new GUILayout.HorizontalScope()) {
                                        EditorGUIUtility.labelWidth = labelWidth;
                                        EditorGUILayout.PropertyField(colorsNumProp, new GUIContent("Gradient Keys"));
                                    }

                                    using (new GUILayout.HorizontalScope()) {
                                        EditorGUILayout.PropertyField(colorAProp, GUIContent.none);
                                        if (colorsNumProp.intValue > 1) {
                                            EditorGUILayout.PropertyField(colorBProp, GUIContent.none);
                                            if (colorsNumProp.intValue > 2) {
                                                EditorGUILayout.PropertyField(colorCProp, GUIContent.none);
                                            }
                                        }
                                    }

                                    if (colorsNumProp.intValue > 1) {
                                        using (new EditorGUILayout.HorizontalScope()) {
                                            using (new EditorGUI.DisabledScope(true)) {
                                                // when disabled alpha channel is divided by 2, so to keep it at 1, multiply alpha
                                                GUI.color = new Color(1, 1, 1, 2);
                                                if (colorsNumProp.intValue > 1) {
                                                    EditorGUILayout.GradientField(gradient, GUILayout.Height(24));
                                                }
                                            }

                                            EditorGUILayout.PropertyField(gradientRotationProp, GUIContent.none);
                                            gradientRotationProp.intValue =
                                                Mathf.RoundToInt(gradientRotationProp.intValue / 45) * 45;
                                        }
                                    }
                                }

                                EditorGUILayout.PropertyField(imageMode);
                                if (imageMode.intValue == 3) {
                                    // Tiled mode
                                    using (new EditorGUILayout.HorizontalScope()) {
                                        EditorGUILayout.PropertyField(tileXProp, new GUIContent("X"));
                                        EditorGUILayout.PropertyField(tileYProp, new GUIContent("Y"));
                                    }

                                }
                                EditorGUILayout.PropertyField(imageType);
                                if (imageType.intValue == 0) {
                                    // sprite
                                    using (new EditorGUILayout.HorizontalScope()) {
                                        EditorGUILayout.PropertyField(spriteProp, new GUIContent("Image"));
                                        if (GUILayout.Button("Set Native Size")) {
                                            foreach (var t in targets) {
                                                Undo.RecordObject(t, "Set Native Size");
                                                (t as RoundedRect).SetNativeSize();
                                            }
                                        }
                                    }
                                }
                                else if (imageType.intValue == 1) {
                                    // texture - deprecated
                                    EditorGUILayout.HelpBox(
                                        "To add an image use Sprite instead. Use Texture for RenderTexture.",
                                        MessageType.Warning);
                                    EditorGUILayout.PropertyField(imageProp, new GUIContent("Image"));
                                }


                            }

                            EditorGUILayout.Space();
                        }

                        // reset
                        EditorGUIUtility.labelWidth = defaultLabelWidth;
                        GUI.color = guiColor;

                        //radius setting
                        if (showInactive) {
                            GUI.contentColor = new Color(1, 1, 1, 1.0f);
                        }

                        using (new EditorGUILayout.HorizontalScope(EditorStyles.helpBox)) {
                            showRadiusSettings = EditorGUILayout.Foldout(showRadiusSettings, "Corners", true);
                        }

                        if (showRadiusSettings) {
                            {
                                using (new GUILayout.VerticalScope()) {
                                    using (new GUILayout.HorizontalScope()) {
                                        EditorGUIUtility.labelWidth = labelWidth;
                                        using (new EditorGUI.DisabledScope(independentCornersProp.boolValue)) {
                                            EditorGUILayout.PropertyField(radiusProp, new GUIContent("Radius"));
                                        }

                                        EditorGUILayout.PropertyField(independentCornersProp, GUIContent.none);
                                    }

                                    if (independentCornersProp.boolValue) {
                                        using (new GUILayout.VerticalScope()) {
                                            EditorGUIUtility.labelWidth = 25;
                                            using (new GUILayout.HorizontalScope()) {
                                                EditorGUILayout.PropertyField(radiusTLProp, new GUIContent("TL"));
                                                EditorGUILayout.PropertyField(radiusTRProp, new GUIContent("TR"));
                                            }

                                            using (new GUILayout.HorizontalScope()) {
                                                EditorGUILayout.PropertyField(radiusBLProp, new GUIContent("BL"));
                                                EditorGUILayout.PropertyField(radiusBRProp, new GUIContent("BR"));
                                            }
                                        }
                                    }
                                }
                            }

                            EditorGUILayout.Space();
                        }

                        // reset
                        EditorGUIUtility.labelWidth = defaultLabelWidth;

                        //border setting
                        using (new EditorGUILayout.HorizontalScope(EditorStyles.helpBox)) {
                            showBorderSettings = EditorGUILayout.Foldout(showBorderSettings, "Border", true);
                        }

                        if (showBorderSettings) {
                            {
                                using (new GUILayout.VerticalScope()) {
                                    EditorGUIUtility.labelWidth = labelWidth;
                                    EditorGUILayout.PropertyField(borderProp, new GUIContent("Border Size"));
                                    EditorGUILayout.PropertyField(borderTypeProp, new GUIContent("Border Type"));
                                    EditorGUILayout.PropertyField(borderColorProp, new GUIContent("Border Color"));
                                    EditorGUILayout.PropertyField(outlineOffsetProp, new GUIContent("Border Offset"));
                                }
                            }

                            EditorGUILayout.Space();
                        }

                        using (new EditorGUILayout.HorizontalScope(EditorStyles.helpBox)) {
                            showGlowSettings = EditorGUILayout.Foldout(showGlowSettings, "Glow", true);
                        }

                        if (showGlowSettings) {
                            {
                                using (new GUILayout.VerticalScope()) {
                                    EditorGUIUtility.labelWidth = labelWidth;
                                    EditorGUILayout.PropertyField(glowColorProp, new GUIContent("Color"));
                                    using (new GUILayout.HorizontalScope()) {}
                                    EditorGUILayout.PropertyField(glowSpreadProp, new GUIContent("Spread"));
                                    EditorGUILayout.PropertyField(glowBlurProp, new GUIContent("Blur"));
                                    using (new GUILayout.HorizontalScope()) {
                                        EditorGUILayout.PropertyField(glowOffsetXProp, new GUIContent("X Offset"));
                                        EditorGUILayout.PropertyField(glowOffsetYProp, new GUIContent("Y Offset"));

                                    }
                                }
                            }

                            EditorGUILayout.Space();
                        }

                        // reset
                        EditorGUIUtility.labelWidth = defaultLabelWidth;

                        // drawExtraSettings must be enabled in the editor code for a component.
                        if (drawExtraSettings) {
                            using (new EditorGUILayout.HorizontalScope(EditorStyles.helpBox)) {
                                showExtraSettings = EditorGUILayout.Foldout(showExtraSettings, "Extras", true);
                            }

                            if (showExtraSettings) {
                                using (new GUILayout.VerticalScope()) {
                                    EditorGUIUtility.labelWidth = labelWidth;
                                    DrawExtraProperties();
                                }
                            }
                        }
                    }
                }

                if (EditorGUI.EndChangeCheck()) {
                    serializedObject.ApplyModifiedProperties();
                    foreach (var t in targets) {
                        EditorUtility.SetDirty(t);
                        (t as RoundedRect).UpdateProperties();
                    }
                }
            }

            protected virtual void DrawExtraProperties() {
                EditorGUILayout.PropertyField(materialProp);
                if (GUILayout.Button("Restore Defaut Material"))
                {
                    foreach (var t in targets) {
                        EditorUtility.SetDirty(t);
                        (t as RoundedRect).RestoreDefaultMaterial();
                    }
                }
            }

            protected void DrawScriptHeader() {
                using (new EditorGUI.DisabledScope(true)) {
                    EditorGUILayout.PropertyField(serializedObject.FindProperty("m_Script"));
                }
            }

            protected virtual void DrawAppearanceHeader(int labelWidth = 110) {
                using (new EditorGUILayout.HorizontalScope()) {
                    EditorGUIUtility.labelWidth = labelWidth;
                    EditorGUILayout.LabelField("APPEARANCE", EditorStyles.miniBoldLabel, GUILayout.Width(labelWidth));
                    AddToHeaderInline();
                }
            }

            protected virtual void AddToHeaderInline() {
                // to add extra buttons in the header
            }

            Vector3 mouseDownStart;
            private float RadiusOnClickDown;
            private bool clickDown;
            private Vector3 constraintDir;
            private int clickedHandleID = -1;

            private Color EditColor = new Color(0.08627451f, 0.5686275f, 0.9176471f, 1);

            private Vector3[] _scaleMatrix = {
                new Vector3(-1, 1, 1),
                new Vector3(1, 1, 1),
                new Vector3(1, -1, 1),
                new Vector3(-1, -1, 1),
            };

            protected virtual void OnSceneGUI() {
                if (ToolManager.activeToolType == typeof(LayoutHandlesTool) && !independentCornersProp.boolValue) {
                    DrawCornerRadiusHandles();
                }
            }

            private void DrawCornerRadiusHandles() {
                var roundedRectWidth = _roundedRect.Width.GetMeters(_roundedRect.UnitsContext);
                var roundedRectHeight = _roundedRect.Height.GetMeters(_roundedRect.UnitsContext);
                var minHeightWidth = Mathf.Min(roundedRectWidth, roundedRectHeight);

                // handle events
                if (Event.current.type == EventType.MouseDown && Event.current.button == 0) {
                    if (!Event.current.control
                        && !Event.current.alt
                        && HandleUtility.nearestControl >= 0 && HandleUtility.nearestControl < 4) {
                        // if needed, store the starting point of the mouse drag in a member variable
                        mouseDownStart = Event.current.mousePosition;

                        clickDown = true;
                        RadiusOnClickDown = _roundedRect.Radius.GetMeters(_roundedRect.UnitsContext);
                        clickedHandleID = HandleUtility.nearestControl;

                        constraintDir =
                            HandleUtility.nearestControl == 0 ? new Vector3(-1, 1, 0) :
                            HandleUtility.nearestControl == 1 ? new Vector3(1, 1, 0) :
                            HandleUtility.nearestControl == 2 ? new Vector3(1, -1, 0) :
                            HandleUtility.nearestControl == 3 ? new Vector3(-1, -1, 0) : Vector3.zero;

                        // if you don't do this, you could click multiple objects at once
                        Event.current.Use();
                    }
                }
                else {
                    clickedHandleID = -1;
                }

                if (Event.current.type == EventType.MouseUp && Event.current.button == 0) {
                    clickDown = false;
                }

                if (Event.current.type == EventType.MouseDrag) {
                    if (clickDown) {
                        // if click down we compute the drag distance and apply it as corner radius

                        float dragDist = -HandleUtility.CalcLineTranslation(mouseDownStart, Event.current.mousePosition,
                            _roundedRect.transform.position, constraintDir) + RadiusOnClickDown;

                        var radius = Mathf.Clamp(dragDist, 0, minHeightWidth * 0.5f);

                        radius = Event.current.shift ? Mathf.Round(radius * 100) / 100 : radius;

                        if (radius < 0.0001f) {
                            radius = 0;
                        }

                        Undo.RecordObject(_roundedRect, "Rounded Rect Radius");
                        _roundedRect._radius.SetFromMeters(_roundedRect.UnitsContext, radius);
                        _roundedRect.UpdateRepresentation();
                        Event.current.Use();

                    }
                }

                DrawRadiusHandles();


            }

            private void DrawRadiusHandles() {
                DrawCornerRadiusHandle(0, _roundedRect);
                DrawCornerRadiusHandle(1, _roundedRect);
                DrawCornerRadiusHandle(2, _roundedRect);
                DrawCornerRadiusHandle(3, _roundedRect);
            }


            private void DrawCornerRadiusHandle(int controlID, RoundedRect roundedRect) {

                var width = _roundedRect.Width.GetMeters(_roundedRect.UnitsContext);
                var height = _roundedRect.Height.GetMeters(_roundedRect.UnitsContext);
                var minHeightWidth = Mathf.Min(width, height);
                var normalizedRadius = _roundedRect.Radius.GetMeters(_roundedRect.UnitsContext) /
                    minHeightWidth * 2;
                var offset = minHeightWidth * 0.05f;

                var clampX = Mathf.Clamp((width - height) / 2, 0, Single.MaxValue);
                var clampY = Mathf.Clamp((height - width) / 2, 0, Single.MaxValue);
                var HandleOffset = Vector3.Lerp(
                    new Vector3(width, height, 0) * 0.5f - new Vector3(offset, offset, 0),
                    new Vector3(clampX, clampY, 0),
                    normalizedRadius);

                var handlePosition = Vector3.Scale(HandleOffset, _scaleMatrix[controlID]);

                var worldHandlePosition = roundedRect.transform.TransformPoint(handlePosition);

                var size = HandleUtility.GetHandleSize(worldHandlePosition) * OCLayoutPreferences.HandlesSize;


                float distFromMouse = HandleUtility.DistanceToCircle(worldHandlePosition, size);
                var fillColor = clickDown ? OCLayoutPreferences.HandlesNeighborColor : Color.white;

                // draw the handle,
                Handles.color = clickDown
                    ? OCLayoutPreferences.HandlesNeighborColor
                    : OCLayoutPreferences.HandlesEditColor;
                //Handles.CylinderHandleCap(controlId , handlePosition, Quaternion.identity, size, EventType.Repaint);
                Handles.CircleHandleCap(controlID, worldHandlePosition, Quaternion.identity, size, EventType.Repaint);
                Handles.DrawWireDisc(worldHandlePosition, _roundedRect.transform.forward, size, 5);
                Handles.color = fillColor;
                Handles.DrawSolidDisc(worldHandlePosition, _roundedRect.transform.forward, size);


                HandleUtility.AddControl(controlID, distFromMouse);


                Debug.Log("Draw ... " + clickDown);


            }
        }
#endif

        #endregion
    }

}
