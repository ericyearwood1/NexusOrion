namespace CTRL.Utils
{

  public static class StringExtensions
  {
    public static byte[] ToByteArray(this string str)
    {
      return System.Text.Encoding.UTF8.GetBytes(str);
    }

    public static string ToUTF8String(this byte[] bytes)
    {
      return System.Text.Encoding.UTF8.GetString(bytes, 0, bytes.Length);
    }
  }

}
