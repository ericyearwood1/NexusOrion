using System.Collections;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [RequireComponent(typeof(AudioSource))]
    public class SpatialAudioOneShot : MonoBehaviour
    {
        [HideInInspector] public AudioSource Source;

        // Start is called before the first frame update
        void Start()
        {
            Source = GetComponent<AudioSource>();
            StartCoroutine(PlayRoutine());
        }

        IEnumerator PlayRoutine()
        {
            Source.Play();
            yield return new WaitForSeconds(Source.clip.length >= 1f ? Source.clip.length : 1f);
            Destroy(gameObject);
        }
    }
}
