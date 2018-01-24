package org.usfirst.frc.team5806.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

public class FileReader {
	public ArrayList<Double> left;
	public ArrayList<Double> right;
	
	public FileReader(String fileName) {
		left = new ArrayList<Double>();
		right = new ArrayList<Double>();
		File file = new File(fileName);
		Scanner input = null;
		try {
			input = new Scanner(file);
		} catch (FileNotFoundException ex) {
			System.out.println("Cannot open " + fileName + ".");
			System.exit(1);
		}
		
		String breakString = "______________________________";
		int i = 0;
		
		while (input.hasNext()) {
			//System.out.println(input.nextString());
			if (input.nextLine().equals(breakString)) {
				System.out.println("a");
				i++;
			}
			if (i == 1) {
				while (input.hasNextDouble()) {
					left.add(input.nextDouble());
				}
			} else if (i == 2) {
				while (input.hasNextDouble()) {
					right.add(input.nextDouble());
				}
			}
 		}
		
		/*
        String line = null;
        
        FileReader fileReader = new FileReader(fileName);
        while (fileReader.ready()) {
        	if (fileReader.read() == '_' && fileReader.ready() && fileReader.read() != '_') {
        		break;
        	}	
        }
        StringBuffer numberStr("");
        while(fileReader.ready()) {
        	while (true) {
        		char c = fileReader.read();
        		if (c != ' ' && c != '\n') {
        			numberStr.append(c);
        		} else if (c != ' ') {
        			left.add(Double.parseDouble(numberStr.toString()));
        			
        		}
        	}
        	
        }
        */
	}
	
	public static void main(String args[]) {
		FileReader reader = new FileReader("TestFile");
		System.out.println(reader.right.get(11));
	}

}