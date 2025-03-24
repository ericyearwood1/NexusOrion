// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using UnityEngine;

namespace ARDS.Utils
{
    public class FigmaSpringSolver {
        float m_w0;
        float m_zeta;
        float m_wd;
        float m_A;
        float m_B;

        public FigmaSpringSolver(float mass, float stiffness, float damping, float initialVelocity)
        {
            m_w0 = Mathf.Sqrt(stiffness / mass);
            m_zeta = damping / (2 * Mathf.Sqrt(stiffness * mass));

            if (m_zeta < 1) {
                // Under-damped.
                m_wd = m_w0 * Mathf.Sqrt(1 - m_zeta * m_zeta);
                m_A = 1;
                m_B = (m_zeta * m_w0 + -initialVelocity) / m_wd;
            } else {
                // Critically damped (ignoring over-damped case for now).
                m_A = 1;
                m_B = -initialVelocity + m_w0;
            }
        }

        public float Solve(float t)
        {
            if (m_zeta < 1) {
                // Under-damped
                t = Mathf.Exp(-t * m_zeta * m_w0) * (m_A * Mathf.Cos(m_wd * t) + m_B * Mathf.Sin(m_wd * t));
            } else {
                // Critically damped (ignoring over-damped case for now).
                t = (m_A + m_B * t) * Mathf.Exp(-t * m_w0);
            }

            // Map range from [1..0] to [0..1].
            return 1 - t;
        }

    };
}
