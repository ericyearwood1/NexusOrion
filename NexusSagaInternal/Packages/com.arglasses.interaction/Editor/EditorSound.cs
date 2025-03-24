using System.IO;
using System.Media;

namespace ARGlasses.Interaction
{
    public static class EditorSound
    {
        public static void PlaySuccess()
        {
            PlaySound("success.wav");
        }

        public static void PlayFail()
        {
            PlaySound("fail.wav");
        }

        private static void PlaySound(string filename)
        {
            var repoPackagesRoot = ExternalProcess.GetRepoPackageFolder("com.arglasses.interaction");
            var soundPath = Path.Combine(repoPackagesRoot, "Editor/Sounds", filename);
            using (var soundPlayer = new SoundPlayer(soundPath))
            {
                soundPlayer.Play();
            }
        }

    }
}
