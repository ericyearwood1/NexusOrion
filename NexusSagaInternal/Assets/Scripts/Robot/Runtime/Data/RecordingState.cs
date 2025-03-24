namespace Robot.Runtime.Data
{
    public enum RecordingState
    {
        None,
        Ready,
        Started,
        Recording,
        Complete,
        Transcribed,
        AwaitingTranscriptionConfirm,
        Cancelled,
        Error,
        Accepted,
        Silent,
        Cancelling,
        Hack
    }
}