using System.Collections.Generic;

namespace CTRL.Data
{
  /// <summary>
  /// The ProfileStatusStream emits this as a message
  /// Represents the currently selected profile as well
  /// as a list of available profiles.
  /// </summary>
#pragma warning disable IDE1006
  [System.Serializable]
  public struct ProfileStatus
  {

    /// <summary>
    /// A Capability represents the availability of a stream, and if/when it was personalized or trained
    /// </summary>
    [System.Serializable]
    public struct Capability
    {
      public string name;
      public bool available;
      public bool trainable;
      public double? trained_at;
    }

    /// <summary>
    /// A profile is a set of trained/personalized models representing a user of
    /// the CTRL-kit Armband.
    /// </summary>
    [System.Serializable]
    public struct Profile
    {
      public string name;
      public List<Capability> capabilities;
    }

    public Profile active_profile;
    public List<Profile> available_profiles;
  }
#pragma warning restore IDE1006

  /// The Profile Status stream returns the currently selected profile, as well
  /// as a list of available profiles. Currently the UnitySDK does not support
  /// switching profiles.
  public class ProfileStatusStream : OutputStreamHandle<ProfileStatus>
  {
    protected override string defaultStreamName => "profile_status";
  }
}
