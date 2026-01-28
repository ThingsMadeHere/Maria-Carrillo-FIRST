package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.Random;

public class SoundPlayer extends SubsystemBase {
    private static SoundPlayer instance;
    private final Random random = new Random();
    private final File soundsDir;
    private File[] soundFiles;
    
    private SoundPlayer() {
        soundsDir = new File(Filesystem.getDeployDirectory(), "sounds");
        refreshSoundFiles();
    }
    
    public static SoundPlayer getInstance() {
        if (instance == null) {
            instance = new SoundPlayer();
        }
        return instance;
    }
    
    public void refreshSoundFiles() {
        if (!soundsDir.exists() || !soundsDir.isDirectory()) {
            soundFiles = new File[0];
            System.err.println("Sounds directory not found at: " + soundsDir.getAbsolutePath());
            return;
        }
        
        soundFiles = soundsDir.listFiles((dir, name) -> 
            name.toLowerCase().endsWith(".wav") || 
            name.toLowerCase().endsWith(".mp3") ||
            name.toLowerCase().endsWith(".aiff")
        );
        
        if (soundFiles == null) {
            soundFiles = new File[0];
        }
        
        System.out.println("Found " + soundFiles.length + " sound files in " + soundsDir.getAbsolutePath());
    }
    
    public void playRandomSound() {
        if (soundFiles.length == 0) {
            System.err.println("No sound files found to play!");
            return;
        }
        
        File soundFile = soundFiles[random.nextInt(soundFiles.length)];
        try {
            // Note: You'll need to implement the actual sound playing logic here
            // This is a placeholder that just logs which sound would be played
            System.out.println("Playing sound: " + soundFile.getName());
            
            // Example of how to play the sound (uncomment and implement based on your audio library):
            // AudioPlayer player = new AudioPlayer(soundFile);
            // player.play();
            
        } catch (Exception e) {
            System.err.println("Error playing sound: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    public int getSoundCount() {
        return soundFiles.length;
    }
}
