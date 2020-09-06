/*
 *  MIT License
 *
 *  Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

package com.github.mittyrobotics.jni;/*
 *  MIT License
 *
 *  Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

import edu.wpi.first.wpiutil.RuntimeLoader;

import java.io.File;
import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;

public class DrakeJNI {
    static boolean libraryLoaded = false;
    static RuntimeLoader<DrakeJNI> loader = null;

    static {
        loadFromFilepath(new File(
                "build\\libs\\tkoLibrariesCppDriver\\shared\\windowsx86-64\\release\\TkoLibrariesCppDriver.dll")
                .getAbsolutePath());
        loadFromFilepath(new File("build\\libs\\tkoLibrariesCpp\\shared\\windowsx86-64\\release\\TkoLibrariesCpp.dll")
                .getAbsolutePath());
//      try {
//        loader = new RuntimeLoader<>("TkoLibrariesCppDriver", RuntimeLoader.getDefaultExtractionRoot(), DrakeJNI.class);
//        loader.loadLibrary();
//      } catch (IOException ex) {
//        ex.printStackTrace();
//        System.exit(1);
//      }
//      libraryLoaded = true;
//    }
        //    if (!libraryLoaded) {
    }

    /**
     * Force load the library.
     *
     * @throws IOException thrown if the native library cannot be found
     */
    public static synchronized void forceLoad() throws IOException {
        if (libraryLoaded) {
            return;
        }
        loader = new RuntimeLoader<>("com.github.mittyrobotics.jni.DrakeJNI", RuntimeLoader.getDefaultExtractionRoot(),
                DrakeJNI.class);
        loader.loadLibrary();
        libraryLoaded = true;
    }

    public static void loadFromFilepath(String filePath) {
        System.load(filePath);
    }

    public static native double initialize();

    public static native double[] discreteAlgebraicRiccatiEquationJNI(double[] A, double[] B, double[] Q, double[] R,
                                                                      int states, int inputs);

    public static void main(String[] args) {
        System.out.println(
                discreteAlgebraicRiccatiEquationJNI(new double[]{1}, new double[]{1}, new double[]{1}, new double[]{1},
                        1, 1));
    }

    public static class Helper {
        private static AtomicBoolean extractOnStaticLoad = new AtomicBoolean(true);

        public static boolean getExtractOnStaticLoad() {
            return extractOnStaticLoad.get();
        }

        public static void setExtractOnStaticLoad(boolean load) {
            extractOnStaticLoad.set(load);
        }
    }
}
