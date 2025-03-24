namespace CTRL.Data
{
  /// A continuous stream of the raw EMG data recorded by the CTRL-kit Armband.
  /// Note: This stream is emitted at approximately 2kHz, and re-batched by
  /// CTRL-R to 50Hz, and then re-batched again in the SDK if your FPS
  /// is less than 50Hz. Using this stream might increase memory pressure dramatically.
  public class EMGStream : OutputStreamHandle<float[]>
  {
    protected override string defaultStreamName => "raw_emg";
  }
}
