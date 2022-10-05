package com.example.taxi.ui.home.provider.information_user

import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.databinding.ItemImgInsideCarBinding
import com.gun0912.tedpermission.provider.TedPermissionProvider

class InformationUserAdapter: RecyclerView.Adapter<InformationUserAdapter.ProviderViewHolder>() {
    var photoList = mutableListOf<String>()

    fun setListData(data: MutableList<String>){
        photoList = data
    }

    fun updateList(list: MutableList<String>){
        this.photoList = list
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ProviderViewHolder {
        return ProviderViewHolder(
            ItemImgInsideCarBinding.inflate(
                LayoutInflater.from(parent.context),
                parent,
                false
            )
        )
    }

    override fun onBindViewHolder(holder: ProviderViewHolder, position: Int) {
        holder.bind(photoList[position])
    }

    override fun getItemCount(): Int {
        return photoList.size
    }

    class ProviderViewHolder(private val binding: ItemImgInsideCarBinding) :
        RecyclerView.ViewHolder(binding.root) {
        fun bind(data: String) {
            Glide.with(TedPermissionProvider.context)
                .load(data)
                .into(binding.imageItemMyPageAlbumImg)
        }

    }

}