package com.vendor.jni;

import java.io.File;
import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpiutil.RuntimeLoader;

public class VendorJNI {
  static boolean libraryLoaded = false;
  static RuntimeLoader<VendorJNI> loader = null;

  public static class Helper {
    private static AtomicBoolean extractOnStaticLoad = new AtomicBoolean(true);

    public static boolean getExtractOnStaticLoad() {
      return extractOnStaticLoad.get();
    }

    public static void setExtractOnStaticLoad(boolean load) {
      extractOnStaticLoad.set(load);
    }
  }

  static {
    if (Helper.getExtractOnStaticLoad()) {
      try {
        loader = new RuntimeLoader<>("Vendor", RuntimeLoader.getDefaultExtractionRoot(), VendorJNI.class);
        loader.loadLibrary();
      } catch (IOException ex) {
//        ex.printStackTrace();
        loadFromFilepath(new File("build\\libs\\vendorDriver\\shared\\windowsx86-64\\release\\VendorDriver.dll").getAbsolutePath());
        loadFromFilepath(new File("build\\libs\\vendor\\shared\\windowsx86-64\\release\\Vendor.dll").getAbsolutePath());

      }
      libraryLoaded = true;
    }
  }

  /**
   * Force load the library.
   * @throws java.io.IOException thrown if the native library cannot be found
   */
  public static synchronized void forceLoad() throws IOException {
    if (libraryLoaded) {
      return;
    }
    loader = new RuntimeLoader<>("VendorJNI", RuntimeLoader.getDefaultExtractionRoot(), VendorJNI.class);
    loader.loadLibrary();
    libraryLoaded = true;
  }

  public static void loadFromFilepath(String filePath){
    System.load(filePath);
  }

  public static native double initialize();

  public static void main(String[] args) {
    System.out.println(initialize());
  }
}
