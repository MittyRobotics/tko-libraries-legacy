/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.motion.jna;

import com.sun.jna.Native;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;

import java.io.File;

/**
 * Cpp JNA library.
 * <p>
 * Instructions: When you first download the repository, open the <code>cpp-util</code> c++ project in a c++ editor.
 * Then build the CMakeLists.txt. This will generate a .dll or .so file for your machine that the JNA can access.
 */
public interface CppUtilJNA extends com.sun.jna.win32.StdCallLibrary {
    public static File libFile = new File("cpp-util\\library-builds\\" + System.mapLibraryName("libtko-libraries-cpp-util"));

    CppUtilJNA INSTANCE = (CppUtilJNA) Native.load(libFile.getAbsolutePath(),
            CppUtilJNA.class);

    Pointer discreteAlgebraicRiccatiEquation(double[] A, double[] B, double[] Q, double[] R, int states, int inputs,
                                             PointerByReference valsRef, IntByReference numValsRef);

    void cleanup(Pointer p);
}