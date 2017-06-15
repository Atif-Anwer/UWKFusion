using log4net;

namespace UWKFusion.Logging
{
    class Logging
    {
        private static readonly ILog log = LogManager.GetLogger(typeof(Logging));
        private static readonly Logging logging = new Logging();

        private Logging()
        {
            log4net.Config.XmlConfigurator.Configure();
        }

        public static Logging Instance
        {
            get { return logging; }
        }

        public void Info(string message)
        {
            log.Info(message);
        }

        public void Warning(string message)
        {
            log.Warn(message);
        }

        public void Debug(string message)
        {
            log.Debug(message);
        }

        public void Error(string message)
        {
            log.Error(message);
        }

        public void Error(System.Exception exc, string message)
        {
            log.Error(message, exc);
        }

    }
}
