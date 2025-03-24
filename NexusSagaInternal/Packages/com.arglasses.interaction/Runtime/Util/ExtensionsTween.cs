// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#if DOTWEEN_ENABLED
using DG.Tweening;
using DG.Tweening.Core;
using DG.Tweening.Plugins.Options;
#endif

using OSIG.Tools.Layout;
using ProtoKit.GraphicBase;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class ExtensionsTween
    {
#if DOTWEEN_ENABLED
        public static TweenerCore<float, float, FloatOptions> DOAlpha(this RoundedRect target, float endValue,
            float duration)
        {
            if (endValue > 1) endValue = 1;
            else if (endValue < 0) endValue = 0;
            var t = DOTween.To(() => target.ColorA.a, a => target.SetColorA(target.ColorA.WithAlpha(a)), endValue,
                duration);
            t.SetTarget(target);
            return t;
        }

        public static TweenerCore<float, float, FloatOptions> DOAlpha(this CanvasGroup target, float endValue,
            float duration)
        {
            if (endValue > 1) endValue = 1;
            else if (endValue < 0) endValue = 0;
            var t = DOTween.To(() => target.alpha, x => target.alpha = x, endValue, duration);
            t.SetTarget(target);
            return t;
        }

        public static TweenerCore<float, float, FloatOptions> DOBorderAlpha(this RoundedRect target, float endValue, float duration)
        {
            if (endValue > 1) endValue = 1;
            else if (endValue < 0) endValue = 0;
            var t = DOTween.To(() => target.BorderColor.a, x => target.BorderColor = target.BorderColor.WithAlpha(x), endValue, duration);
            t.SetTarget(target);
            return t;
        }

        public static TweenerCore<float, float, FloatOptions> DOBorderSize(this RoundedRect rr, float endValue, float duration)
        {
            if (endValue < 0) endValue = 0;
            void Setter(float f)
            {
                rr.SetBorder(f);
                // rr.SetAllDirty();
            }

            var t = DOTween.To(() => rr.Border.GetPixels(rr.UnitsContext), Setter, endValue, duration);
            t.SetTarget(rr);
            return t;
        }

        public static TweenerCore<Color, Color, ColorOptions> DOColorA(this RoundedRect target, Color endValue, float duration)
        {
            var t = DOTween.To(() => target.ColorA, x => target.ColorA = x, endValue, duration);
            t.SetTarget(target);
            return t;
        }

        public static TweenerCore<float, float, FloatOptions> DOWidth(this RoundedRect target, float endValue, float duration)
        {
            return target.GetComponent<OCLayoutSize>().DOWidth(endValue, duration);
        }

        public static TweenerCore<float, float, FloatOptions> DOWidth(this OCLayoutSize target, float endValue, float duration)
        {
            if (endValue < 0) endValue = 0;
            var t = DOTween.To(() => target.Width.Size.GetPixels(target.UnitsContext), target.SetWidth, endValue, duration);
            t.SetTarget(target);
            return t;
        }

        public static TweenerCore<float, float, FloatOptions> DOHeight(this RoundedRect target, float endValue, float duration)
        {
            return target.GetComponent<OCLayoutSize>().DOHeight(endValue, duration);
        }

        public static TweenerCore<float, float, FloatOptions> DOHeight(this OCLayoutSize target, float endValue, float duration)
        {
            if (endValue < 0) endValue = 0;
            var t = DOTween.To(() => target.Height.Size.GetPixels(target.UnitsContext), target.SetHeight, endValue, duration);
            t.SetTarget(target);
            return t;
        }

        public static TweenerCore<float, float, FloatOptions> DOMoveY(
            this OCInsetAttachData target,
            float endValue,
            float duration)
        {
            TweenerCore<float, float, FloatOptions> t = DOTween.To(() => target.GetYOffset().RawValue,
                target.SetYOffset, endValue, duration);
            t.SetTarget<Tweener>(target);
            return t;
        }

        // public static TweenerCore<float, float, FloatOptions> DOFloat(this Material target, float endValue, int propertyID, float duration)
        // {
        //     if (!target.HasProperty(propertyID)) {
        //         if (Debugger.logPriority > 0) Debugger.LogMissingMaterialProperty(propertyID);
        //         return null;
        //     }
        //     TweenerCore<float, float, FloatOptions> t = DOTween.To(() => target.GetFloat(propertyID), x => target.SetFloat(propertyID, x), endValue, duration);
        //     t.SetTarget(target);
        //     return t;
        // }
#endif

    }
}
