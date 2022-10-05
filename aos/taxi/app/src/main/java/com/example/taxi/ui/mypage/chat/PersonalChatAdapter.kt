package com.example.taxi.ui.mypage.chat

import android.view.Gravity
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.bumptech.glide.request.RequestOptions
import com.example.taxi.R
import com.example.taxi.data.dto.mypage.ChatModel
import com.example.taxi.di.ApplicationClass

class PersonalChatAdapter: RecyclerView.Adapter<PersonalChatAdapter.MessageViewHolder>() {
    private var commentList = mutableListOf<ChatModel.CommentModel>()
    lateinit var onItemClickListener: (View, String) -> Unit

    fun setListData(data: MutableList<ChatModel.CommentModel>){
        commentList = data
    }

    fun updateList(list: MutableList<ChatModel.CommentModel>){
        this.commentList = list
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): MessageViewHolder {
        val view : View = LayoutInflater.from(parent.context).inflate(R.layout.item_message, parent, false)

        return MessageViewHolder(view)
    }

    override fun onBindViewHolder(holder: MessageViewHolder, position: Int) {
//        holder.bind(commentList[position])

        holder.textView_message.textSize = 20F
        holder.textView_message.text = commentList[position].message
        holder.textView_time.text = commentList[position].time

        var userName = ApplicationClass.userId
        // email Set으로 들어가면 오류가 생기므로 .과 @를 없앤다.
        userName = userName!!.replace(".", "").replace("@", "")

        var carName: String = ""

        if(ApplicationClass.prefs.carName != null){
            carName = ApplicationClass.prefs.carName!!
        }

        if(commentList[position].userName.equals(userName) || commentList[position].userName.equals(carName)){ // 본인 채팅
            holder.textView_message.setBackgroundResource(R.drawable.ic_chat_right)
            holder.textView_name.visibility = View.INVISIBLE
            holder.layout_destination.visibility = View.INVISIBLE
            holder.layout_main.gravity = Gravity.RIGHT
        }else{ // 상대방 채팅
            Glide.with(holder.itemView.context)
                .load( ApplicationClass.prefs.destinationUserImg)
                .apply(RequestOptions().circleCrop())
                .into(holder.imageView_profile)
            holder.textView_name.text = ApplicationClass.prefs.destinationUserName
            holder.layout_destination.visibility = View.VISIBLE
            holder.textView_name.visibility = View.VISIBLE
            holder.textView_message.setBackgroundResource(R.drawable.ic_chat_left)
            holder.layout_main.gravity = Gravity.LEFT
        }
    }

    inner class MessageViewHolder(view: View) : RecyclerView.ViewHolder(view) {
        val imageView_profile: ImageView = view.findViewById(R.id.image_messageItem_profile)
        val textView_name: TextView = view.findViewById(R.id.text_messageItem_name)
        val textView_message: TextView = view.findViewById(R.id.text_messageItem_message)
        val textView_time: TextView = view.findViewById(R.id.text_messageItem_time)
        val layout_main: LinearLayout = view.findViewById(R.id.layout_messageItem_main)
        val layout_destination: LinearLayout = view.findViewById(R.id.layout_messageItem_destination)
    }

    override fun getItemCount(): Int {
        return commentList.size
    }
}