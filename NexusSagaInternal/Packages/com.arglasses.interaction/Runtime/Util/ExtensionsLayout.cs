using OSIG.Tools.Layout;
using OSIG.Tools.Units;
using ProtoKit.GraphicBase;
using TMPro;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class ExtensionsLayout
    {
        public static RoundedRect WithAlpha(this RoundedRect rr, float a)
        {
            rr.SetColorA(rr.ColorA.WithAlpha(a));
            return rr;
        }
        public static RoundedRect WithBorderAlpha(this RoundedRect rr, float a)
        {
            rr.SetBorderColor(rr.BorderColor.WithAlpha(a));
            return rr;
        }
        public static RoundedRect WithBorderSizePx(this RoundedRect rr, float sizePx)
        {
            rr.SetBorder(sizePx);
            rr.SetAllDirty();
            return rr;
        }

        public static RoundedRect LerpAlpha(this RoundedRect rr, float target, float t)
        {
            var c = rr.ColorA;
            rr.SetColorA(c.WithAlpha(Mathf.Lerp(c.a, target, t)));
            return rr;
        }

        public static T SetCenterFill<T>(this T layout, float elevation = 0, float paddingX = 0, float paddingY = 0, float inflation = 0, float min = 0)
            where T : OCLayoutComponentBase
        {
            if (paddingX == 0) paddingX = inflation;
            if (paddingY == 0) paddingY = inflation;

            OCInsetAttachData proxyAd = layout.GetAttachData<OCInsetAttachData>();
            if (!proxyAd)
            {
                OCNullAttachData nullAd = layout.GetComponent<OCNullAttachData>();
                if (nullAd) return layout;

                proxyAd = layout.gameObject.AddComponent<OCSizeAttachData>();
            }

            proxyAd.SetPaddingFront(-elevation);
            proxyAd.SetPaddingLeft(-paddingX);
            proxyAd.SetPaddingRight(-paddingX);
            proxyAd.SetPaddingTop(-paddingY);
            proxyAd.SetPaddingBottom(-paddingY);
            proxyAd.Set3DAlignment(OCLayoutPadding.Alignment.CenterFill);

            if (min > 0)
            {
                if (layout is OCLayoutSize layoutSize)
                {
                    layoutSize.SetSizeMode(OCLayoutSize.SizeMode.Min);
                    layoutSize.SetSize(Vector3.one * min);
                }
                else
                {
                    Debug.LogError("min is only supported for OCLayoutSize");
                }
            }

            proxyAd.SetDirty();
            return layout;
        }

        public static Vector3 CurrentSizeSafe(this OCLayoutComponentBase layout)
        {
            return layout.GetComponent<RectTransform>().rect.size;
        }

        public static void FreezeSize(this OCLayoutSize layout)
        {
            var rect = CurrentSizeSafe(layout);
            var width = rect.x.AsMeters();
            var height = rect.y.AsMeters();
            layout.SetWidthMode(OCLayoutSize.SizeMode.Override);
            layout.SetHeightMode(OCLayoutSize.SizeMode.Override);
            layout.SetWidth(width);
            layout.SetHeight(height);
        }

        public static bool IsRelative(this OCInsetAttachData attachData) => attachData.GetPaddingType() == OCLayoutPadding.PaddingType.Relative;
        public static bool IsAbsolute(this OCInsetAttachData attachData) => attachData.GetPaddingType() == OCLayoutPadding.PaddingType.Absolute;

        public static TextMeshProUGUI WithAlpha(this TextMeshProUGUI text, float alpha)
        {
            text.color = text.color.WithAlpha(alpha);
            return text;
        }
    }
}
