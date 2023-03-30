using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using NAudio.Wave;
using System.IO;
using System.Collections.Concurrent;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Globalization;
using System.Collections.ObjectModel;
using System.Threading;

namespace AudioSplitter
{

  public class PropertyChangedBase : INotifyPropertyChanged
  {
    public event PropertyChangedEventHandler PropertyChanged;
    // Create the OnPropertyChanged method to raise the event
    // The calling member's name will be used as the parameter.
    protected void OnPropertyChanged([CallerMemberName] string name = null)
    {
      PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }
  }

  /// <summary>
  /// Interaction logic for MainWindow.xaml
  /// </summary>
  public partial class MainWindow : Window, INotifyPropertyChanged
  {
    public event PropertyChangedEventHandler PropertyChanged;
    // Create the OnPropertyChanged method to raise the event
    // The calling member's name will be used as the parameter.
    protected void OnPropertyChanged([CallerMemberName] string name = null)
    {
      PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }

    public struct Division
    {
      public SolidColorBrush Color { get; set; }
      public SolidColorBrush TextColor { get; set; }
      public int Index { get; set; }
    }

    public class Bookmark : PropertyChangedBase
    {
      private string m_label;
      private int m_index;
      private bool m_include = true;

      public long Position { get; set; }
      public int Index
      {
        get => m_index;
        set
        {
          m_index = value;
          IsOdd = (m_index % 2 == 0);
          OnPropertyChanged(nameof(IsOdd));
        }
      }
      public string Label
      {
        get => m_label;
        set
        {
          m_label = value;
          OnPropertyChanged(nameof(Label));
        }
      }
      public bool IsOdd { get; set; }
      public bool Include
      {
        get => m_include;
        set
        {
          m_include = value;
          OnPropertyChanged(nameof(Include));
        }
      }
    }

    private IWavePlayer m_wavePlayer = null;
    private Mp3FileReader m_reader = null;
    private string m_mp3FilePath = "";
    private string m_outputPath = "";
    private long m_position = 0;
    private long m_minPosition = 0;
    private long m_maxPosition = 0;

    public List<int> Parts { get; set; }
    public int SelectedParts { get; set; } = 10;
    public long MinPosition
    {
      get => m_minPosition;
      set
      {
        m_minPosition = value;
        OnPropertyChanged(nameof(MinPosition));
      }
    }

    public long MaxPosition
    {
      get => m_maxPosition;
      set
      {
        m_maxPosition = value;
        OnPropertyChanged(nameof(MaxPosition));
      }
    }

    public long Position
    {
      get => m_position;
      set
      {
        m_position = value;
        double frac = (double)Position / (m_reader.Length - 1) * 100;
        string label = frac.ToString("n2");
        PositionLabel = Position.ToString() + " (" + label + "%)";
        OnPropertyChanged(nameof(Position));
        OnPropertyChanged(nameof(PositionLabel));
      }
    }

    public string PositionLabel { get; set; } = "";
    public List<Division> Divisions { get; set; } = new List<Division>();
    public ObservableCollection<Bookmark> Bookmarks { get; set; } = new ObservableCollection<Bookmark>();

    public string OutputPath
    {
      get => m_outputPath;
      set
      {
        m_outputPath = value;
        OnPropertyChanged(nameof(OutputPath));
      }
    }

    public string MP3FilePath
    {
      get => m_mp3FilePath;
      set
      {
        m_mp3FilePath = value;
        OnPropertyChanged(nameof(MP3FilePath));
      }
    }

    public MainWindow()
    {
      InitializeComponent();
      BtnPause.IsEnabled = false;
      BtnPlay.IsEnabled = false;
      PreviewKeyDown += MainWindow_PreviewKeyDown;
      Parts = Enumerable.Range(0, 51).ToList();
      Bookmarks.CollectionChanged += Bookmarks_CollectionChanged;
      this.DataContext = this;
      this.Closing += MainWindow_Closing;
      Task.Run(PlayerAudio);
    }

    private void MainWindow_Closing(object sender, CancelEventArgs e)
    {
      SaveBookmarks();
    }

    private void PlayerAudio()
    {
      while (true)
      {
        try
        {
          if(m_wavePlayer != null)
          {
            this.Dispatcher.Invoke(() =>
            {
              if(m_wavePlayer != null && m_wavePlayer.PlaybackState == PlaybackState.Playing)
                 Position = m_reader.Position;
              BtnPause.IsEnabled = (m_wavePlayer.PlaybackState == PlaybackState.Playing);
              BtnPlay.IsEnabled = !BtnPause.IsEnabled;
            });
           }
        }
        catch (Exception) { }
        Thread.Sleep(100);
      }
    }

    private void Bookmarks_CollectionChanged(object sender, System.Collections.Specialized.NotifyCollectionChangedEventArgs e)
    {
      if (m_wavePlayer == null)
        return;

      int count = 1;
      foreach (Bookmark bk in Bookmarks)
      {
        bk.Index = count - 1;
        double frac = (double)bk.Position / (m_reader.Length - 1) * 100;
        string label = frac.ToString("n2");
        bk.Label = "[" + count.ToString() + "] " + bk.Position.ToString() + " (" + label + "%)";
        count++;
      }
    }

    private void MainWindow_PreviewKeyDown(object sender, KeyEventArgs e)
    {
      if (m_wavePlayer == null)
        return;

      if (e.Key == Key.Enter)
      {
        Bookmark bk = new Bookmark();
        bk.Position = this.Position;
        Bookmarks.Add(bk);
        return;
      }

      if (e.Key != Key.Left && e.Key != Key.Right)
        return;
      long newPos = (e.Key == Key.Left) ? m_reader.Position - 1000000 : m_reader.Position + 1000000;
      if (newPos < 0)
        newPos = m_reader.Length - 1;
      if (newPos >= m_reader.Length)
        newPos = 0;

      m_wavePlayer.Pause();
      m_reader.Position = newPos;
      m_wavePlayer.Play();
      e.Handled = true;
    }

    private void SetPosition(double fraction)
    {
      if (m_wavePlayer == null)
        return;
      m_wavePlayer.Pause();
      long newPos = (long)((m_reader.Length - 1) * fraction);
      if (newPos > m_reader.Length - 1)
        newPos = m_reader.Length - 1;
      m_reader.Position = newPos;
      Position = m_reader.Position;
      m_wavePlayer.Play();
    }

    private void Timeline_PreviewMouseDoubleClick(object sender, MouseButtonEventArgs e)
    {
      if (m_wavePlayer == null)
        return;
      Point p = e.GetPosition(Timeline);
      double fraction = p.X / Timeline.ActualWidth;
      SetPosition(fraction);
    }

    private void Timeline_PreviewMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
      if (m_wavePlayer == null)
        return;
      if (!(Keyboard.IsKeyDown(Key.LeftCtrl) || Keyboard.IsKeyDown(Key.RightCtrl)))
        return;
      Point p = e.GetPosition(Timeline);
      double fraction = p.X / Timeline.ActualWidth;
      SetPosition(fraction);
    }

    private void BtnPause_Click(object sender, RoutedEventArgs e)
    {
      if (m_wavePlayer == null)
        return;
      m_wavePlayer.Pause();
      BtnPlay.IsEnabled = true;
      BtnPause.IsEnabled = false;
    }

    private void BtnPlay_Click(object sender, RoutedEventArgs e)
    {
      if (m_wavePlayer == null)
        return;
      m_wavePlayer.Play();
    }

    private void BtnBookmark_Click(object sender, RoutedEventArgs e)
    {
      if (m_wavePlayer == null)
        return;
      Bookmark bk = new Bookmark();
      bk.Position = this.Position;
      Bookmarks.Add(bk);
    }

    private void NumPartsCombo_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
      if (m_wavePlayer == null)
        return;
      Divisions.Clear();
      OnPropertyChanged(nameof(Divisions));
      List<Division> divisions = new List<Division>();
      OnPropertyChanged(nameof(Divisions));
      HashSet<byte> reds = new HashSet<byte>();
      HashSet<byte> greens = new HashSet<byte>();
      HashSet<byte> blues = new HashSet<byte>();
      var rand = new Random();
      byte[] rgb = new byte[3];
      for (int i = 0; i < SelectedParts; i++)
      {
        Division division = new Division();
        while(true)
        {
          rand.NextBytes(rgb);
          if (!reds.Contains(rgb[0]) && !greens.Contains(rgb[1]) && !blues.Contains(rgb[2]))
          {
            reds.Add(rgb[0]);
            greens.Add(rgb[1]);
            blues.Add(rgb[2]);
            break;
          }
        }
        division.Color = new SolidColorBrush(Color.FromRgb(rgb[0], rgb[1], rgb[2]));
        division.TextColor = new SolidColorBrush(Color.FromRgb((byte)(255 - rgb[0]), (byte)(255 - rgb[1]), (byte)(255 - rgb[2])));
        division.Index = i + 1;
        divisions.Add(division);
      }
      Divisions = divisions;
      OnPropertyChanged(nameof(Divisions));
    }

    private void ListBoxItem_PreviewMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
      if (m_wavePlayer == null)
        return;

      ListBoxItem item = sender as ListBoxItem;
      if (item == null)
        return;
      Bookmark bk = item.DataContext as Bookmark;
      if (bk == null)
        return;

      SetPosition((double)bk.Position / (m_reader.Length - 1));
    }

    private void ListBoxItem_PreviewKeyDown(object sender, KeyEventArgs e)
    {
      if (m_wavePlayer == null)
        return;

      ListBoxItem item = sender as ListBoxItem;
      if (item == null)
        return;
      Bookmark bk = item.DataContext as Bookmark;
      if (bk == null)
        return;

      if (Bookmarks.Contains(bk))
        Bookmarks.Remove(bk);
    }

    private void BtnOpenFile_Click(object sender, RoutedEventArgs e)
    {
      using (System.Windows.Forms.OpenFileDialog openFileDialog = new System.Windows.Forms.OpenFileDialog())
      {
        openFileDialog.InitialDirectory = "C:/Code";
        openFileDialog.Filter = "mp3 files (*.mp3)|*.mp3";
        //openFileDialog.FilterIndex = 2;
        openFileDialog.RestoreDirectory = true;
        if (openFileDialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
        {
          //Get the path of specified file
          var filePath = openFileDialog.FileName;
          if (!File.Exists(filePath) || filePath == MP3FilePath)
            return;

          SaveBookmarks();
          if(m_wavePlayer != null)
          {
            m_wavePlayer.Pause();
            m_wavePlayer.Dispose();
            m_reader.Dispose();
          }
          MP3FilePath = filePath;
          m_reader = new Mp3FileReader(filePath);
          m_wavePlayer = new WaveOut();
          m_wavePlayer.Init(m_reader);
          m_wavePlayer.Play();
          MinPosition = 0;
          MaxPosition = m_reader.Length - 1;
          Position = m_reader.Position;
          OpenBookmarks();
        }
      }
    }

    private void BtnBrowseOutputDir_Click(object sender, RoutedEventArgs e)
    {
      using (System.Windows.Forms.FolderBrowserDialog openFolderDialog = new System.Windows.Forms.FolderBrowserDialog())
      {
        openFolderDialog.SelectedPath = "C:/Code/cd";
        openFolderDialog.Description = "Selet output folder";
        //openFileDialog.FilterIndex = 2;
        if (openFolderDialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
        {
          OutputPath = openFolderDialog.SelectedPath;
        }
      }
    }

    private void SaveBookmarks()
    {
      if (m_wavePlayer == null || Bookmarks.Count == 0 || string.IsNullOrEmpty(MP3FilePath) || !File.Exists(MP3FilePath))
        return;

      string bookmarksPath = MP3FilePath + ".bmk";
      if (File.Exists(bookmarksPath))
        File.Delete(bookmarksPath);
      StreamWriter sw = new StreamWriter(bookmarksPath);
      foreach(Bookmark bk in Bookmarks)
      {
        sw.WriteLine(bk.Position.ToString() + "," + bk.Include.ToString());
      }
      sw.Flush();
      sw.Close();
    }

    private void OpenBookmarks()
    {
      Bookmarks.Clear();
      string bookmarksPath = MP3FilePath + ".bmk";
      if (!File.Exists(bookmarksPath))
        return;

      StreamReader sr = new StreamReader(bookmarksPath);
      while (sr.Peek() >= 0)
      {
        string line = sr.ReadLine();
        if (line == null)
          continue;

        string[] splits = line.Split(new char[] { ',' });
        if (splits.Length < 1)
          continue;

        long position;
        if(long.TryParse(splits[0], out position))
        {
          Bookmark bk = new Bookmark();
          bk.Position = position;
          bk.Index = Bookmarks.Count;
          if(splits.Length > 1)
          {
            bool include = true;
            if (bool.TryParse(splits[1], out include))
              bk.Include = include;
          }
          Bookmarks.Add(bk);
        }

      }
      sr.Close();
    }

    private void SplitMP3_Click(object sender, RoutedEventArgs e)
    {
      if (m_wavePlayer == null || Bookmarks.Count == 0 || string.IsNullOrEmpty(MP3FilePath) || !File.Exists(MP3FilePath))
        return;

      string outpath = OutputPath;
      if(string.IsNullOrEmpty(outpath) || !Directory.Exists(outpath))
      {
        //outpath = System.IO.Path.GetDirectoryName(System.Windows.Forms.Application.ExecutablePath);
        outpath = System.IO.Path.GetDirectoryName(MP3FilePath);
      }

      List<long> splitPostions = new List<long>();
      List<bool> includes = new List<bool>();
      includes.Add(false);
      splitPostions.Insert(0, 0);

      foreach (Bookmark bk in Bookmarks)
      {
        if (bk.Position == 0)
          continue;

        splitPostions.Add(bk.Position);
        includes.Add(bk.Include);
      }

      if (MaxPosition - splitPostions[splitPostions.Count - 1] > 1000)
      {
        splitPostions.Add(MaxPosition + 1);
        includes.Add(true);
      }

      m_wavePlayer.Pause();
      try
      {
        string basename = System.IO.Path.GetFileNameWithoutExtension(MP3FilePath);
        TimeSpan totalSpan = TimeSpan.FromMilliseconds(0);
        int currentCD = 1;
        long bytesPerSample = m_reader.WaveFormat.BitsPerSample / 8 * m_reader.WaveFormat.Channels;
        for (int i = 1; i < splitPostions.Count(); i++)
        {
          if (!includes[i])
            continue;

          long start = splitPostions[i - 1];
          long end = splitPostions[i] - 1;
          string partName = basename + "_" + i.ToString().PadLeft(2,'0') + ".mp3";
          string fullPath = System.IO.Path.Combine(outpath, partName);
          if (File.Exists(fullPath))
            File.Delete(fullPath);
          FileStream output = new FileStream(fullPath, FileMode.CreateNew);
          if ((output.Position == 0) && (m_reader.Id3v2Tag != null))
          {
            output.Write(m_reader.Id3v2Tag.RawData, 0, m_reader.Id3v2Tag.RawData.Length);
          }
          Mp3Frame frame;
          m_reader.Position = start;
          long len = 0;
          while ((frame = m_reader.ReadNextFrame()) != null && m_reader.Position < end)
          {
            len += frame.SampleCount * bytesPerSample;
            output.Write(frame.RawData, 0, frame.RawData.Length);
          }
          output.Flush();
          output.Close();

          TimeSpan partSpan = TimeSpan.FromSeconds((double)len / (double)m_reader.WaveFormat.AverageBytesPerSecond);
          totalSpan = totalSpan.Add(partSpan);
          if (totalSpan.TotalMinutes > 79.9999)
          {
            totalSpan = TimeSpan.FromMilliseconds(0);
            currentCD++;
          }
          string cdPath = System.IO.Path.Combine(outpath, basename + "_CD_" + currentCD.ToString());
          if (!Directory.Exists(cdPath))
            Directory.CreateDirectory(cdPath);
          string destFile = System.IO.Path.Combine(cdPath, partName);
          if (File.Exists(destFile))
            File.Delete(destFile);
          File.Move(fullPath, destFile);
        }
      }
      finally
      {
        m_reader.Position = Position;
        if (!BtnPlay.IsEnabled)
          m_wavePlayer.Play();
      }
    }
  }
}
