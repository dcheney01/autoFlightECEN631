from moviepy.editor import VideoFileClip, CompositeVideoClip, concatenate_videoclips, TextClip

def add_text_to_video(video_path, text_elements, output_path):
    '''
    Adds text to a video.
    Inputs:
        video_path: the path to the video
        text_elements: a list of dictionaries, each containing the text to be added and its position
            text: the text to be added
            relative_position: the position of the text, a tuple of relative coordinates (relative to left and top)
            fontsize: the fontsize of the text
            color: the color of the text
            font: the font of the text
        output_path: the path to the output video
    Example usage:
        video_path = "video.mp4"
        text_elements = [{"text": "Hello", "relative_position": (0.5, 0.5)}, 
                         {"text": "World", "relative_position": (0.5, 0.4)}]
        add_text_to_video(video_path, text_elements, "output.mp4")
    '''
    # Load video clip
    video_clip = VideoFileClip(video_path)

    # Create a list to hold TextClip objects
    text_clips = []

    # Create TextClip objects for each text element
    for text_info in text_elements:
        text = text_info['text']
        fontsize = text_info.get('fontsize', 30)
        color = text_info.get('color', 'white')
        position = text_info.get('relative_position', ('0.5', '0.8'))
        font = text_info.get('font', 'Helvetica-Light')

        text_clip = TextClip(text, fontsize=fontsize, color=color, font=font)
        text_clip = text_clip.set_pos((position), relative=True).set_duration(video_clip.duration)

        text_clips.append(text_clip)

    # Composite the video clip and the text clips
    final_clip = CompositeVideoClip([video_clip] + text_clips)

    # Write the result to a file
    final_clip.write_videofile(output_path, codec="libx264", fps=24)

    # Close video clips
    video_clip.close()
    final_clip.close()

def combine_video_list(video_paths, output_name):
    '''
    Combines a list of videos into one video.
    Inputs:
        video_paths: a list of paths to the videos to be combined
        output_name: the name of the output video
    Example usage:
        video_paths = ["video1.mp4", "video2.mp4"]
        combine_video_list(video_paths, "output.mp4")
    '''
    # Load video clips
    video_clips = [VideoFileClip(video_path) for video_path in video_paths]

    # Combine video clips
    final_clip = concatenate_videoclips(video_clips)

    # Write the result to a file
    final_clip.write_videofile(output_name, codec="libx264", fps=24)

    # Close video clips
    for video_clip in video_clips:
        video_clip.close()
    final_clip.close()


if __name__=="__main__":
    video_names = ["StraightLineManager", "FilletManager", "DubinsManager"]
    video_folder = "videos/chp11/"
    
    extension = ".mp4"
    
    # First add text to all videos (based on video name)
    for video in video_names:
        video_path = video_folder + video + extension
        text_elements = [{"text": video, "relative_position": (0.01, 0.11)}]
        output_path = video_folder + video + "_text" + extension
        add_text_to_video(video_path, text_elements, output_path)

    # Now combine all videos into one final video
    video_paths = [f"{video_folder+video}_text{extension}" for video in video_names]
    output_path = f"{video_folder}00final_video.mp4"
    combine_video_list(video_paths, output_path)
    

    