package com.example.taxi.ui.mypage.chatbot

import android.view.Gravity
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.LinearLayout
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.example.taxi.databinding.ItemChatBotBinding

class ChatBotMessageAdapter: RecyclerView.Adapter<ChatBotMessageAdapter.ChatBotMessageMessageViewHolder>() {
    private var commentList = mutableListOf<String>()
    lateinit var onMessageClickListener: (View, String) -> Unit

    fun setListData(data: MutableList<String>){
        commentList = data
    }

    fun updateList(list: MutableList<String>){
        this.commentList = list
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ChatBotMessageMessageViewHolder {
        return ChatBotMessageMessageViewHolder(
            ItemChatBotBinding.inflate(
                LayoutInflater.from(parent.context),
                parent,
                false
            )
        ).apply {
            bindOnItemClickListener(onMessageClickListener)
        }
    }

    override fun onBindViewHolder(holder: ChatBotMessageMessageViewHolder, position: Int) {
        holder.text_message.text = commentList[position]
    }

    inner class ChatBotMessageMessageViewHolder(private val binding: ItemChatBotBinding)
        : RecyclerView.ViewHolder(binding.root) {
        val text_message = binding.textChatBotMessage

        fun bindOnItemClickListener(onMessageClickListener: (View, String) -> Unit ) {
            text_message.setOnClickListener {
                onMessageClickListener(it, text_message.text.toString())
            }
        }
    }

    override fun getItemCount(): Int {
        return commentList.size
    }
}