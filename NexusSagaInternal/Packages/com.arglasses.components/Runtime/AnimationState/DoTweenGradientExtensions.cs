#if DOTWEEN_ENABLED
using DG.Tweening;
using DG.Tweening.Core;
using DG.Tweening.Plugins.Options;
#endif
using ProtoKit.UI;
using UnityEngine;

namespace ARGlasses.Components
{
    public static class DoTweenGradientExtensions
    {
#if DOTWEEN_ENABLED

        public static Sequence DOGradientColor(this PKUIPanel target, Gradient gradient, float duration)
        {
            Sequence s = DOTween.Sequence();
            GradientColorKey[] colors = gradient.colorKeys;
            int len = colors.Length;
            for (int i = 0; i < len; ++i) {
                GradientColorKey c = colors[i];
                if (i == 0 && c.time <= 0) {
                    target.color = c.color;
                    continue;
                }
                float colorDuration = i == len - 1
                    ? duration - s.Duration(false) // Verifies that total duration is correct
                    : duration * (i == 0 ? c.time : c.time - colors[i - 1].time);
                s.Append(target.DOColor(c.color, colorDuration).SetEase(Ease.Linear));
            }
            s.SetTarget(target);
            return s;
        }

        public static TweenerCore<Color, Color, ColorOptions> DOColor(this PKUIPanel target, Color endValue, float duration)
        {
            TweenerCore<Color, Color, ColorOptions> t = DOTween.To(() => target.ColorA, x => target.ColorA = x, endValue, duration);
            t.SetTarget(target);
            return t;
        }
#endif
    }
}
