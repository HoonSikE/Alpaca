package com.example.taxi.ui.mypage.chatbot

import android.view.Gravity
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.LinearLayout
import android.widget.TextView
import androidx.core.view.isInvisible
import androidx.core.view.isVisible
import androidx.core.view.marginTop
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.example.taxi.databinding.ItemChatBotListBinding

class ChatBotAdapter: RecyclerView.Adapter<ChatBotAdapter.ChatBotMessageViewHolder>() {
    private var commentList = mutableListOf<String>()
    lateinit var onMessageClickListener: (View, String) -> Unit
    private lateinit var chatBotMessageAdapterAdapter: ChatBotMessageAdapter

    fun setListData(data: MutableList<String>){
        commentList = data
    }

    fun updateList(list: MutableList<String>){
        this.commentList = list
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ChatBotMessageViewHolder {
//        val view : View = LayoutInflater.from(parent.context).inflate(R.layout.item_chat_bot_list, parent, false)

        return ChatBotMessageViewHolder(
            ItemChatBotListBinding.inflate(
                LayoutInflater.from(parent.context),
                parent,
                false
            )
        ).apply {
            bindOnItemClickListener(onMessageClickListener)
        }
    }

    override fun onBindViewHolder(holder: ChatBotMessageViewHolder, position: Int) {
        if(commentList[position].substring(0 until 1) != "나"){
            holder.layout_main.gravity = Gravity.LEFT
            holder.textView_message.text = commentList[position]
            holder.textView_message.setBackgroundResource(R.drawable.ic_chat_left)
            if(commentList[position].substring(0 until 2) == "아래" || position == 0){
                holder.image_chat_bot.visibility = View.VISIBLE
                holder.text_chat_bot_name.visibility = View.VISIBLE
            }else{
                holder.image_chat_bot.visibility = View.INVISIBLE
                holder.text_chat_bot_name.visibility = View.INVISIBLE
            }
        }else {
            holder.layout_main.gravity = Gravity.RIGHT
            holder.textView_message.text = commentList[position].substring(2 until commentList[position].length)
            holder.textView_message.setBackgroundResource(R.drawable.ic_chat_right)
            holder.image_chat_bot.visibility = View.INVISIBLE
            holder.text_chat_bot_name.visibility = View.INVISIBLE
        }

    }

    inner class ChatBotMessageViewHolder(private val binding: ItemChatBotListBinding)
        : RecyclerView.ViewHolder(binding.root) {
        val layout_main = binding.layoutChatBotMain
//        val textView_message = binding.textChatBotListContent
        val textView_message = binding.textChatBotListContent
        val image_chat_bot = binding.imageChatBotList
        val text_chat_bot_name = binding.textChatBotListName

        fun bindOnItemClickListener(onMessageClickListener: (View, String) -> Unit ) {
            textView_message.setOnClickListener {
                onMessageClickListener(it, textView_message.text.toString())
            }
        }
    }

    override fun getItemCount(): Int {
        return commentList.size
    }
}