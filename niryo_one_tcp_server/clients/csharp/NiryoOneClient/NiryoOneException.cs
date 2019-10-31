using System;

namespace NiryoOneClient
{
    public class NiryoOneException : Exception
    {
        public NiryoOneException(string reason)
        {
            Reason = reason;
        }

        public string Reason { get; }
    }
}