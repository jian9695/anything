using System;
using System.Linq;
using System.Collections.Generic;
using System.Windows.Forms;
using System.ComponentModel.Composition;
using System.ComponentModel.Composition.Hosting;
using NAudio.Wave;
using System.IO;

namespace AudioFileInspector
{
  static class Program
  {
    /// <summary>
    /// The main entry point for the application.
    /// </summary>
    [STAThread]
    static int Main(string[] args)
    {

      //if (File.Exists("C:/Code/CD/test.mp3"))
      //  File.Delete("C:/Code/CD/test.mp3");
      //FileStream output = new FileStream("C:/Code/CD/test.mp3", FileMode.CreateNew);

      //Mp3FileReader reader = new Mp3FileReader("C:/Code/CD/PolyGram-20.mp3");
      //if ((output.Position == 0) && (reader.Id3v2Tag != null))
      //{
      //  output.Write(reader.Id3v2Tag.RawData, 0, reader.Id3v2Tag.RawData.Length);
      //}
      //IWavePlayer wavePlayer = new WaveOut();
      //wavePlayer.Init(reader);
      //wavePlayer.Play();
      //reader.Position = 500000000;
      //while (true)
      //{
      //  if (reader.Position % 10000 == 0)
      //    System.Diagnostics.Debug.WriteLine(reader.Position.ToString());
      //}
      //Mp3Frame frame;
      //int count = 0;
  
      //while ((frame = reader.ReadNextFrame()) != null)
      //{
      //  if (count > 10000 && count < 20000)
      //    output.Write(frame.RawData, 0, frame.RawData.Length);
      //  count++;
      //}
      //output.Flush();
      //output.Close();
      Application.EnableVisualStyles();
      Application.SetCompatibleTextRenderingDefault(false);

      var catalog = new AssemblyCatalog(System.Reflection.Assembly.GetExecutingAssembly());
      var container = new CompositionContainer(catalog);
      var inspectors = container.GetExportedValues<IAudioFileInspector>();

      if (args.Length > 0)
      {
        if (args[0] == "-install")
        {
          try
          {
            OptionsForm.Associate(inspectors);
            Console.WriteLine("Created {0} file associations", inspectors.Count());
          }
          catch (Exception e)
          {
            Console.WriteLine("Unable to create file associations");
            Console.WriteLine(e.ToString());
            return -1;
          }

          return 0;
        }
        else if (args[0] == "-uninstall")
        {
          try
          {
            OptionsForm.Disassociate(inspectors);
            Console.WriteLine("Removed {0} file associations", inspectors.Count());
          }
          catch (Exception e)
          {
            Console.WriteLine("Unable to remove file associations");
            Console.WriteLine(e.ToString());
            return -1;
          }
          return 0;
        }
      }
      var mainForm = container.GetExportedValue<AudioFileInspectorForm>();
      mainForm.CommandLineArguments = args;
      Application.Run(mainForm);
      return 0;
    }
  }
}